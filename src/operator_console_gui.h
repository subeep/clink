// operator_console_gui.h
//
// Modern Operator Console for System 3 with:
//   - Top Status & Telemetry Header
//   - Left Sidebar Navigation Tabs (2.4 GHz, 5.1 GHz, 5.8 GHz, All/Combined)
//   - Focused High-Resolution Waveform & FFT Plot Views
//   - Real-Time Channel Diagnostics (Peak Voltage, SNR/Power, Carrier Metrics)

#pragma once

#include <complex>
#include <string>
#include <vector>

#include "audio_player.h"
#include "fft_processor.h"
#include "multichannel_demux.h"
#include "net_protocol.h"
#include "osm_slippy_map.h"
#include "ring_buffer.h"
#include "status_provider.h"
#include "synthetic_telemetry.h"

struct GLFWwindow;

enum class ConsoleTab
{
    k24GHz        = 0,
    k51GHz        = 1,
    k58GHz        = 2,
    kAll          = 3,
    kTerrestrial  = 4,
    kAirMaritime  = 5,
    kReplayDos    = 6,
    kGnss         = 7
};

class OperatorConsoleGui
{
public:
    OperatorConsoleGui(IMonitorStatus& status, MultichannelDemux& demux,
                       double sample_rate_hz, std::string window_title);
    ~OperatorConsoleGui();

    void run();

private:
    bool init_window();
    void shutdown_window();
    void draw_frame();

    void draw_top_status_bar();
    void draw_left_sidebar();
    void draw_main_content();

    void draw_single_channel_view(int channel_idx, const char* title_desc,
                                 double y_min, double y_max, double default_carrier_mhz);

    void draw_all_channels_view();

    // 4 Specialized Domain Views
    void draw_terrestrial_decoder_view();
    void draw_air_maritime_awareness_view();
    void draw_replay_dos_view();
    void draw_gnss_monitor_view();

    // Reusable Custom Visualization Renderers
    void render_waveform_plot(const char* plot_id, IqRingBuffer& ring,
                             std::vector<std::complex<float>>& scratch,
                             std::vector<float>& i_buf, std::vector<float>& q_buf,
                             double y_min, double y_max, float& out_peak_v);

    void render_fft_plot(const char* plot_id, ChannelData& ch_data,
                         std::vector<float>& local_fft_db,
                         std::vector<float>& freq_axis,
                         double default_carrier_mhz,
                         double& last_carrier_mhz,
                         float& out_peak_db, double& out_peak_mhz);

    void render_polar_map_plot(const char* plot_id, float elevation_deg, float azimuth_deg, bool is_active = true);

    void render_clear_tactical_map(const char* plot_id, const ExtendedDomainTelemetry& telem, float width, float height);
    void render_polar_ppi_scope(const char* plot_id, const ExtendedDomainTelemetry& telem, float width, float height);
    void render_stft_waterfall(const char* plot_id, const float* slice_128, float width, float height);
    void render_terrestrial_waterfall(const char* plot_id, float width, float height);
    void render_air_waterfall(const char* plot_id, float width, float height);
    void render_hw_fingerprint_radar(const char* plot_id, const ExtendedDomainTelemetry& telem);

    IMonitorStatus& status_;
    MultichannelDemux& demux_;
    double sample_rate_hz_;
    std::string window_title_;

    GLFWwindow* window_ = nullptr;
    ConsoleTab active_tab_ = ConsoleTab::k24GHz;

    static constexpr size_t kWaveformDisplaySamples = 2000;
    static constexpr size_t kFftSize = 4096;
    static constexpr size_t kHistoryLen20 = 20;

    struct ChannelScratch
    {
        std::vector<std::complex<float>> wave_scratch;
        std::vector<float> i_buf;
        std::vector<float> q_buf;
        std::vector<float> fft_db;
        std::vector<float> fft_freq;
        double last_carrier_mhz{0.0};
        float peak_v{0.0f};
        float peak_db{-120.0f};
        double peak_mhz{0.0};
    };

    ChannelScratch ch_scratch_[4];
    std::vector<float> x_axis_wave_;

    // Historical 20-point Timeline Data for GNSS & Replay-DoS
    std::vector<float> time_axis_20_;
    std::vector<float> gnss_time_power_;
    std::vector<float> gnss_time_agc_;
    std::vector<float> gnss_time_pr_res_;
    std::vector<float> gnss_time_dop_res_;
    std::vector<float> gnss_time_phase_res_;
    std::vector<float> gnss_time_pos_e_;
    std::vector<float> gnss_time_pos_n_;
    std::vector<float> gnss_time_pos_u_;
    std::vector<float> gnss_time_vel_;

    std::vector<float> dos_time_noise_;
    std::vector<float> dos_time_duty_;
    std::vector<float> dos_time_entropy_;
    std::vector<float> dos_time_tdoa_;
    std::vector<float> dos_time_cir_;

    // Waterfalls (Circular buffers)
    static constexpr size_t kStftBins = 128;
    static constexpr size_t kStftHistory = 80;
    std::vector<float> stft_waterfall_matrix_; // kStftHistory * kStftBins
    std::vector<float> terr_waterfall_matrix_; // kStftHistory * kStftBins
    std::vector<float> air_waterfall_matrix_;  // kStftHistory * kStftBins

    // UI Input State & Controls
    bool show_all_grid_ = false;
    float radar_sweep_rad_ = 0.0f;
    uint64_t ui_tick_ = 0;

    double usrp_ui_freq1_ = 1090.000;
    double usrp_ui_freq2_ = 162.000;
    double usrp_ui_rate_ = 10.0;
    double usrp_ui_gain_ = 54.0;
    int usrp_ui_proto_ = 0;
    int usrp_ui_sync_ = 0;
    int usrp_ui_crc_ = 0;
    int usrp_ui_ant_ = 0;
    bool usrp_params_applied_ = false;
    float usrp_apply_timer_ = 0.0f;

    char air_search_query_[64] = "";
    char topic_search_query_[64] = "";

    // Tactical Map Geographic Viewport & Layer Controls (Tab 6)
    float map_center_lat_{37.7749f};    // SF Bay Base Station Latitude
    float map_center_lon_{-122.4194f};  // SF Bay Base Station Longitude
    float map_zoom_nmi_{28.0f};         // Viewport radius in nautical miles
    float map_pan_lat_{0.0f};
    float map_pan_lon_{0.0f};
    bool show_air_layer_{true};
    bool show_sea_layer_{true};
    bool show_channels_layer_{true};
    bool show_buoys_layer_{true};
    bool show_airspace_layer_{true};
    bool show_trails_layer_{true};
    bool show_vectors_layer_{true};
    bool show_range_rings_{true};

    // Target Selection & Inspection HUD
    int selected_target_domain_{0}; // 0=None, 1=Air, 2=Sea
    int selected_target_idx_{-1};

    // Audio Player & Visualizer
    AudioPlayer audio_player_;
    float audio_vis_buf_[128];

    // OpenStreetMap (OSM) & Slippy Map Engine
    OsmSlippyMap osm_map_;

    SyntheticTelemetryEngine local_fallback_engine_;
};

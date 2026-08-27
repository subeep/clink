// osm_slippy_map.h
//
// OpenStreetMap (OSM) & Slippy Map Engine for Dear ImGui / OpenGL 3
// Features:
// - Web Mercator (EPSG:3857) projection math (Lat/Lon <-> Tile X/Y <-> Screen Pixels)
// - Multi-provider tile layers (CartoDB Dark Matter, OSM Standard, Esri Satellite, Positron, OpenTopo)
// - Multithreaded async tile downloader with thread pool
// - Persistent local disk cache (~/.cache/usrp_operator_console/tiles/)
// - GPU VRAM LRU OpenGL texture cache (256x256 RGBA)
// - Smooth mouse drag panning, scroll-wheel zooming (centered on cursor)
// - Quick location preset jumping (SF Bay, NY JFK, LAX, London, Tokyo, Singapore)
// - Real-time tactical overlay rendering (ADS-B aircraft, AIS maritime vessels, range rings, radar sweeps)

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>

#include "net_protocol.h"

enum class MapProvider : int
{
    kCartoDark = 0,    // "CartoDB Dark Matter (Tactical C2)"
    kOsmStandard = 1,  // "OpenStreetMap Standard"
    kEsriSatellite = 2,// "Esri World Imagery (Satellite)"
    kCartoPositron = 3,// "CartoDB Positron (Light)"
    kOpenTopo = 4      // "OpenTopoMap (Terrain Contours)"
};

struct TileKey
{
    int provider = 0;
    int z = 0;
    int x = 0;
    int y = 0;

    bool operator==(const TileKey& o) const
    {
        return provider == o.provider && z == o.z && x == o.x && y == o.y;
    }
};

struct TileKeyHash
{
    std::size_t operator()(const TileKey& k) const noexcept
    {
        std::size_t h1 = std::hash<int>{}(k.provider);
        std::size_t h2 = std::hash<int>{}(k.z);
        std::size_t h3 = std::hash<int>{}(k.x);
        std::size_t h4 = std::hash<int>{}(k.y);
        return h1 ^ (h2 << 6) ^ (h3 << 12) ^ (h4 << 18);
    }
};

struct DecodedTile
{
    TileKey key;
    int width = 0;
    int height = 0;
    std::vector<uint8_t> rgba_data;
};

class OsmSlippyMap
{
public:
    OsmSlippyMap();
    ~OsmSlippyMap();

    // Disable copy
    OsmSlippyMap(const OsmSlippyMap&) = delete;
    OsmSlippyMap& operator=(const OsmSlippyMap&) = delete;

    // Viewport & Navigation
    void set_center(double lat, double lon);
    void set_zoom(float zoom);
    void reset_to_sf_bay();
    void set_preset_location(int preset_index);

    double center_lat() const { return center_lat_; }
    double center_lon() const { return center_lon_; }
    float zoom() const { return zoom_; }

    // Map Layer Provider
    void set_provider(MapProvider provider);
    MapProvider provider() const { return current_provider_; }

    // Coordinate Transformations (Web Mercator EPSG:3857)
    ImVec2 geo_to_screen(double lat, double lon) const;
    void screen_to_geo(ImVec2 screen_pt, double& out_lat, double& out_lon) const;

    // Main GUI Render Methods
    // Renders toolbar, map viewport, tile grid, and tactical overlays
    void render_full_map_view(const char* viewport_id, const ExtendedDomainTelemetry& telem,
                             float width, float height,
                             int& selected_contact_index, bool& is_air_selected,
                             bool show_air, bool show_sea, bool show_trails,
                             bool show_vectors, bool show_airspace, bool show_channels,
                             bool show_buoys, bool show_rings, uint32_t ui_tick);

private:
    // Internal Web Mercator Math
    static double lon_to_tile_x(double lon, int z);
    static double lat_to_tile_y(double lat, int z);
    static double tile_x_to_lon(double x, int z);
    static double tile_y_to_lat(double y, int z);

    // Texture & Download Management
    void init_workers(int num_threads = 4);
    void stop_workers();
    void worker_loop();
    void process_completed_textures_on_main_thread();
    void request_tile(const TileKey& key);
    GLuint get_or_request_texture(const TileKey& key);
    std::string get_tile_url(const TileKey& key) const;
    std::string get_tile_cache_path(const TileKey& key) const;

    // Drawing Helpers
    void draw_tactical_contacts(ImDrawList* draw_list, ImVec2 p_min, ImVec2 p_max,
                                const ExtendedDomainTelemetry& telem,
                                int& selected_contact_index, bool& is_air_selected,
                                bool show_air, bool show_sea, bool show_trails,
                                bool show_vectors, bool show_airspace, bool show_channels,
                                bool show_buoys, bool show_rings, uint32_t ui_tick);

    void draw_aircraft_icon(ImDrawList* draw_list, ImVec2 pos, float heading_deg,
                            float size, ImU32 col_fill, ImU32 col_outline);
    void draw_vessel_icon(ImDrawList* draw_list, ImVec2 pos, float heading_deg,
                          float size, ImU32 col_fill, ImU32 col_outline);

    // Viewport State
    double center_lat_ = 37.7749;   // San Francisco Bay
    double center_lon_ = -122.4194;
    float zoom_ = 11.2f;            // Fractional zoom level (3.0f to 18.0f)
    MapProvider current_provider_ = MapProvider::kCartoDark;

    // Last computed viewport screen bounds
    ImVec2 viewport_min_{0.0f, 0.0f};
    ImVec2 viewport_max_{0.0f, 0.0f};
    ImVec2 viewport_size_{0.0f, 0.0f};

    // Cache Directory Path
    std::string cache_root_dir_;

    // OpenGL Texture Cache (Main thread only)
    std::unordered_map<TileKey, GLuint, TileKeyHash> texture_cache_;
    std::deque<TileKey> lru_queue_;
    static constexpr size_t kMaxGpuTextures = 400;

    // Multithreaded Async Downloader
    std::vector<std::thread> workers_;
    std::atomic<bool> stop_flag_{false};

    std::mutex queue_mutex_;
    std::condition_variable cv_download_;
    std::deque<TileKey> pending_downloads_;
    std::unordered_set<TileKey, TileKeyHash> enqueued_keys_;

    std::mutex ready_mutex_;
    std::deque<DecodedTile> ready_tiles_;

    // Interaction & Animation State
    bool is_panning_ = false;
    ImVec2 last_mouse_pos_{0.0f, 0.0f};
    float radar_sweep_angle_deg_ = 0.0f;
};

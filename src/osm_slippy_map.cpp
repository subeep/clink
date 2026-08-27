// osm_slippy_map.cpp
//
// OpenStreetMap (OSM) & Slippy Map Engine for Dear ImGui / OpenGL 3

#include "osm_slippy_map.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

#define STB_IMAGE_IMPLEMENTATION
#include "../third_party/stb_image.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kDeg2Rad = kPi / 180.0;

bool directory_exists(const std::string& path)
{
    struct stat info;
    if (stat(path.c_str(), &info) != 0) return false;
    return (info.st_mode & S_IFDIR) != 0;
}

void make_dir_recursive(const std::string& path)
{
    std::string current;
    for (size_t i = 0; i < path.size(); ++i) {
        current += path[i];
        if (path[i] == '/' && !current.empty()) {
            if (!directory_exists(current)) {
                mkdir(current.c_str(), 0755);
            }
        }
    }
    if (!directory_exists(path)) {
        mkdir(path.c_str(), 0755);
    }
}
} // namespace

OsmSlippyMap::OsmSlippyMap()
{
    const char* home = std::getenv("HOME");
    if (home) {
        cache_root_dir_ = std::string(home) + "/.cache/usrp_operator_console/tiles";
    } else {
        cache_root_dir_ = "/tmp/usrp_operator_console_tiles";
    }
    make_dir_recursive(cache_root_dir_);

    init_workers(4);
}

OsmSlippyMap::~OsmSlippyMap()
{
    stop_workers();

    // Clean up all allocated OpenGL textures
    for (auto& pair : texture_cache_) {
        if (pair.second != 0) {
            glDeleteTextures(1, &pair.second);
        }
    }
    texture_cache_.clear();
}

void OsmSlippyMap::set_center(double lat, double lon)
{
    center_lat_ = std::clamp(lat, -85.0511, 85.0511);
    while (lon > 180.0) lon -= 360.0;
    while (lon < -180.0) lon += 360.0;
    center_lon_ = lon;
}

void OsmSlippyMap::set_zoom(float zoom)
{
    zoom_ = std::clamp(zoom, 3.0f, 18.0f);
}

void OsmSlippyMap::reset_to_sf_bay()
{
    center_lat_ = 37.7749;
    center_lon_ = -122.4194;
    zoom_ = 11.2f;
}

void OsmSlippyMap::set_preset_location(int preset_index)
{
    switch (preset_index) {
    case 0: // San Francisco Bay
        center_lat_ = 37.7749; center_lon_ = -122.4194; zoom_ = 11.2f; break;
    case 1: // New York / JFK
        center_lat_ = 40.6413; center_lon_ = -73.7781; zoom_ = 11.0f; break;
    case 2: // Los Angeles / LAX
        center_lat_ = 33.9416; center_lon_ = -118.4085; zoom_ = 11.0f; break;
    case 3: // London / Dover Strait
        center_lat_ = 51.1279; center_lon_ = 1.3134; zoom_ = 11.0f; break;
    case 4: // Tokyo Bay
        center_lat_ = 35.5494; center_lon_ = 139.7798; zoom_ = 11.0f; break;
    case 5: // Singapore Strait
        center_lat_ = 1.290270; center_lon_ = 103.851959; zoom_ = 11.5f; break;
    default:
        reset_to_sf_bay(); break;
    }
}

void OsmSlippyMap::set_provider(MapProvider provider)
{
    if (current_provider_ != provider) {
        current_provider_ = provider;
    }
}

// -----------------------------------------------------------------------------
// Web Mercator (EPSG:3857) Math
// -----------------------------------------------------------------------------
double OsmSlippyMap::lon_to_tile_x(double lon, int z)
{
    return (lon + 180.0) / 360.0 * (1 << z);
}

double OsmSlippyMap::lat_to_tile_y(double lat, int z)
{
    double lat_rad = lat * kDeg2Rad;
    return (1.0 - std::asinh(std::tan(lat_rad)) / kPi) / 2.0 * (1 << z);
}

double OsmSlippyMap::tile_x_to_lon(double x, int z)
{
    return x / static_cast<double>(1 << z) * 360.0 - 180.0;
}

double OsmSlippyMap::tile_y_to_lat(double y, int z)
{
    double n = kPi - 2.0 * kPi * y / static_cast<double>(1 << z);
    return kRad2Deg * std::atan(0.5 * (std::exp(n) - std::exp(-n)));
}

ImVec2 OsmSlippyMap::geo_to_screen(double lat, double lon) const
{
    int z_int = std::clamp(static_cast<int>(std::floor(zoom_)), 2, 18);
    double scale = std::pow(2.0, zoom_ - z_int);
    double tile_w = 256.0 * scale;

    double center_tx = lon_to_tile_x(center_lon_, z_int);
    double center_ty = lat_to_tile_y(center_lat_, z_int);

    double pt_tx = lon_to_tile_x(lon, z_int);
    double pt_ty = lat_to_tile_y(lat, z_int);

    float sx = viewport_min_.x + viewport_size_.x * 0.5f + static_cast<float>((pt_tx - center_tx) * tile_w);
    float sy = viewport_min_.y + viewport_size_.y * 0.5f + static_cast<float>((pt_ty - center_ty) * tile_w);

    return ImVec2(sx, sy);
}

void OsmSlippyMap::screen_to_geo(ImVec2 screen_pt, double& out_lat, double& out_lon) const
{
    int z_int = std::clamp(static_cast<int>(std::floor(zoom_)), 2, 18);
    double scale = std::pow(2.0, zoom_ - z_int);
    double tile_w = 256.0 * scale;

    double center_tx = lon_to_tile_x(center_lon_, z_int);
    double center_ty = lat_to_tile_y(center_lat_, z_int);

    double delta_x = (screen_pt.x - (viewport_min_.x + viewport_size_.x * 0.5f)) / tile_w;
    double delta_y = (screen_pt.y - (viewport_min_.y + viewport_size_.y * 0.5f)) / tile_w;

    double pt_tx = center_tx + delta_x;
    double pt_ty = center_ty + delta_y;

    out_lon = tile_x_to_lon(pt_tx, z_int);
    out_lat = tile_y_to_lat(pt_ty, z_int);
}

// -----------------------------------------------------------------------------
// Tile Download & Cache Management
// -----------------------------------------------------------------------------
std::string OsmSlippyMap::get_tile_url(const TileKey& key) const
{
    char buf[512];
    switch (static_cast<MapProvider>(key.provider)) {
    case MapProvider::kCartoDark:
        std::snprintf(buf, sizeof(buf), "https://a.basemaps.cartocdn.com/dark_all/%d/%d/%d.png", key.z, key.x, key.y);
        break;
    case MapProvider::kOsmStandard:
        std::snprintf(buf, sizeof(buf), "https://tile.openstreetmap.org/%d/%d/%d.png", key.z, key.x, key.y);
        break;
    case MapProvider::kEsriSatellite:
        std::snprintf(buf, sizeof(buf), "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%d/%d/%d", key.z, key.y, key.x);
        break;
    case MapProvider::kCartoPositron:
        std::snprintf(buf, sizeof(buf), "https://a.basemaps.cartocdn.com/light_all/%d/%d/%d.png", key.z, key.x, key.y);
        break;
    case MapProvider::kOpenTopo:
        std::snprintf(buf, sizeof(buf), "https://tile.opentopomap.org/%d/%d/%d.png", key.z, key.x, key.y);
        break;
    default:
        std::snprintf(buf, sizeof(buf), "https://a.basemaps.cartocdn.com/dark_all/%d/%d/%d.png", key.z, key.x, key.y);
        break;
    }
    return std::string(buf);
}

std::string OsmSlippyMap::get_tile_cache_path(const TileKey& key) const
{
    const char* prov_name = "carto_dark";
    switch (static_cast<MapProvider>(key.provider)) {
    case MapProvider::kCartoDark: prov_name = "carto_dark"; break;
    case MapProvider::kOsmStandard: prov_name = "osm_std"; break;
    case MapProvider::kEsriSatellite: prov_name = "esri_sat"; break;
    case MapProvider::kCartoPositron: prov_name = "carto_light"; break;
    case MapProvider::kOpenTopo: prov_name = "opentopo"; break;
    }

    char buf[512];
    std::snprintf(buf, sizeof(buf), "%s/%s/%d/%d", cache_root_dir_.c_str(), prov_name, key.z, key.x);
    make_dir_recursive(buf);

    std::snprintf(buf, sizeof(buf), "%s/%s/%d/%d/%d.png", cache_root_dir_.c_str(), prov_name, key.z, key.x, key.y);
    return std::string(buf);
}

void OsmSlippyMap::init_workers(int num_threads)
{
    stop_flag_ = false;
    for (int i = 0; i < num_threads; ++i) {
        workers_.emplace_back(&OsmSlippyMap::worker_loop, this);
    }
}

void OsmSlippyMap::stop_workers()
{
    stop_flag_ = true;
    cv_download_.notify_all();
    for (auto& w : workers_) {
        if (w.joinable()) {
            w.join();
        }
    }
    workers_.clear();
}

void OsmSlippyMap::worker_loop()
{
    while (!stop_flag_) {
        TileKey key;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_download_.wait(lock, [&]() {
                return stop_flag_ || !pending_downloads_.empty();
            });
            if (stop_flag_) break;

            key = pending_downloads_.front();
            pending_downloads_.pop_front();
        }

        std::string cache_path = get_tile_cache_path(key);
        std::string tmp_path = cache_path + ".tmp";

        // Check if cached file already exists and is non-empty
        struct stat st;
        bool exists = (stat(cache_path.c_str(), &st) == 0 && st.st_size > 500);

        if (!exists) {
            std::string url = get_tile_url(key);
            char cmd[1024];
            std::snprintf(cmd, sizeof(cmd),
                          "curl -s -m 5 -A \"USRPOperatorConsole/1.0 (contact: operator@example.com)\" \"%s\" -o \"%s\"",
                          url.c_str(), tmp_path.c_str());

            int res = std::system(cmd);
            if (res == 0 && stat(tmp_path.c_str(), &st) == 0 && st.st_size > 500) {
                std::rename(tmp_path.c_str(), cache_path.c_str());
                exists = true;
            } else {
                std::remove(tmp_path.c_str());
            }
        }

        if (exists) {
            int w = 0, h = 0, comp = 0;
            unsigned char* data = stbi_load(cache_path.c_str(), &w, &h, &comp, 4);
            if (data && w > 0 && h > 0) {
                DecodedTile dt;
                dt.key = key;
                dt.width = w;
                dt.height = h;
                dt.rgba_data.assign(data, data + (w * h * 4));
                stbi_image_free(data);

                std::lock_guard<std::mutex> lock(ready_mutex_);
                ready_tiles_.push_back(std::move(dt));
            }
        }
    }
}

void OsmSlippyMap::request_tile(const TileKey& key)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (enqueued_keys_.find(key) == enqueued_keys_.end()) {
        enqueued_keys_.insert(key);
        // Prioritize newly requested tiles by pushing to front
        pending_downloads_.push_front(key);
        cv_download_.notify_one();
    }
}

void OsmSlippyMap::process_completed_textures_on_main_thread()
{
    std::deque<DecodedTile> batch;
    {
        std::lock_guard<std::mutex> lock(ready_mutex_);
        // Process up to 12 textures per frame for smooth 60fps
        size_t count = std::min<size_t>(ready_tiles_.size(), 12);
        for (size_t i = 0; i < count; ++i) {
            batch.push_back(std::move(ready_tiles_.front()));
            ready_tiles_.pop_front();
        }
    }

    for (auto& dt : batch) {
        if (texture_cache_.find(dt.key) == texture_cache_.end()) {
            GLuint tex = 0;
            glGenTextures(1, &tex);
            glBindTexture(GL_TEXTURE_2D, tex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dt.width, dt.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, dt.rgba_data.data());

            texture_cache_[dt.key] = tex;
            lru_queue_.push_back(dt.key);

            // Evict old textures if cache limit exceeded
            while (texture_cache_.size() > kMaxGpuTextures && !lru_queue_.empty()) {
                TileKey old_key = lru_queue_.front();
                lru_queue_.pop_front();
                auto it = texture_cache_.find(old_key);
                if (it != texture_cache_.end()) {
                    if (it->second != 0) {
                        glDeleteTextures(1, &it->second);
                    }
                    texture_cache_.erase(it);
                }
            }
        }
    }
}

GLuint OsmSlippyMap::get_or_request_texture(const TileKey& key)
{
    auto it = texture_cache_.find(key);
    if (it != texture_cache_.end()) {
        return it->second;
    }
    request_tile(key);
    return 0;
}

// -----------------------------------------------------------------------------
// Main Render Loop & Overlays
// -----------------------------------------------------------------------------
void OsmSlippyMap::render_full_map_view(const char* viewport_id, const ExtendedDomainTelemetry& telem,
                                       float width, float height,
                                       int& selected_contact_index, bool& is_air_selected,
                                       bool show_air, bool show_sea, bool show_trails,
                                       bool show_vectors, bool show_airspace, bool show_channels,
                                       bool show_buoys, bool show_rings, uint32_t ui_tick)
{
    process_completed_textures_on_main_thread();

    ImGui::BeginChild(viewport_id, ImVec2(width, height), true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // --- Interactive Toolbar ---
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.0f, 2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));

    // 1. Zoom Controls
    if (ImGui::Button(" ➕ ")) { set_zoom(zoom_ + 0.5f); }
    ImGui::SameLine();
    if (ImGui::Button(" ➖ ")) { set_zoom(zoom_ - 0.5f); }
    ImGui::SameLine();
    if (ImGui::Button("🎯 Center SF")) { reset_to_sf_bay(); }
    ImGui::SameLine();

    // 2. Preset Locations Dropdown
    const char* presets[] = {
        "🌉 SF Bay Metro (37.77°N, -122.42°W)",
        "🗽 New York JFK (40.64°N, -73.78°W)",
        "🌴 Los Angeles LAX (33.94°N, -118.41°W)",
        "🎡 London / Dover (51.12°N, 1.31°E)",
        "🗼 Tokyo Bay (35.55°N, 139.78°E)",
        "⚓ Singapore Strait (1.29°N, 103.85°E)"
    };
    static int current_preset = 0;
    ImGui::SetNextItemWidth(175.0f);
    if (ImGui::Combo("##LocationPresets", &current_preset, presets, IM_ARRAYSIZE(presets))) {
        set_preset_location(current_preset);
    }
    ImGui::SameLine();

    // 3. Map Layer Style Provider Selector
    const char* styles[] = {
        "🌌 CartoDB Dark Matter (Tactical C2)",
        "🗺️ OpenStreetMap Standard",
        "🛰️ Esri World Satellite Imagery",
        "☀️ CartoDB Positron (Light)",
        "⛰️ OpenTopoMap (Topographical)"
    };
    int style_idx = static_cast<int>(current_provider_);
    ImGui::SetNextItemWidth(210.0f);
    if (ImGui::Combo("##MapStyleCombo", &style_idx, styles, IM_ARRAYSIZE(styles))) {
        set_provider(static_cast<MapProvider>(style_idx));
    }

    ImGui::SameLine(0, 10.0f);
    ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f), "Zoom: %.1f | Lat: %.4f° Lon: %.4f°",
                       zoom_, center_lat_, center_lon_);

    ImGui::PopStyleVar(2);
    ImGui::Spacing();

    // --- Viewport Calculation ---
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (avail.x < 50.0f || avail.y < 50.0f) {
        ImGui::EndChild();
        return;
    }

    ImVec2 p_min = ImGui::GetCursorScreenPos();
    ImVec2 p_max = ImVec2(p_min.x + avail.x, p_min.y + avail.y);

    viewport_min_ = p_min;
    viewport_max_ = p_max;
    viewport_size_ = avail;

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->PushClipRect(p_min, p_max, true);

    // --- Interactive Mouse Drag & Scroll Zoom Handling ---
    ImGuiIO& io = ImGui::GetIO();
    bool hovered = ImGui::IsWindowHovered();

    if (hovered) {
        // Drag to Pan
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            ImVec2 delta = io.MouseDelta;
            if (delta.x != 0.0f || delta.y != 0.0f) {
                int z_int = std::clamp(static_cast<int>(std::floor(zoom_)), 2, 18);
                double scale = std::pow(2.0, zoom_ - z_int);
                double tile_w = 256.0 * scale;

                double center_tx = lon_to_tile_x(center_lon_, z_int);
                double center_ty = lat_to_tile_y(center_lat_, z_int);

                center_tx -= delta.x / tile_w;
                center_ty -= delta.y / tile_w;

                center_lon_ = tile_x_to_lon(center_tx, z_int);
                center_lat_ = tile_y_to_lat(center_ty, z_int);
                set_center(center_lat_, center_lon_);
            }
        }

        // Scroll Wheel to Zoom (Centered on Mouse Cursor)
        if (io.MouseWheel != 0.0f) {
            double mouse_lat, mouse_lon;
            screen_to_geo(io.MousePos, mouse_lat, mouse_lon);

            float new_zoom = std::clamp(zoom_ + io.MouseWheel * 0.35f, 3.0f, 18.0f);
            set_zoom(new_zoom);

            // Re-adjust center so mouse remains over the exact same geo coordinate
            int z_int = std::clamp(static_cast<int>(std::floor(zoom_)), 2, 18);
            double scale = std::pow(2.0, zoom_ - z_int);
            double tile_w = 256.0 * scale;

            double mouse_tx = lon_to_tile_x(mouse_lon, z_int);
            double mouse_ty = lat_to_tile_y(mouse_lat, z_int);

            double delta_x = (io.MousePos.x - (p_min.x + avail.x * 0.5f)) / tile_w;
            double delta_y = (io.MousePos.y - (p_min.y + avail.y * 0.5f)) / tile_w;

            double new_center_tx = mouse_tx - delta_x;
            double new_center_ty = mouse_ty - delta_y;

            center_lon_ = tile_x_to_lon(new_center_tx, z_int);
            center_lat_ = tile_y_to_lat(new_center_ty, z_int);
            set_center(center_lat_, center_lon_);
        }
    }

    // --- Base Oceanic Background ---
    draw_list->AddRectFilled(p_min, p_max, IM_COL32(10, 15, 24, 255));

    // --- Render Slippy Map Tiles ---
    int z_int = std::clamp(static_cast<int>(std::floor(zoom_)), 2, 18);
    double scale = std::pow(2.0, zoom_ - z_int);
    double tile_w = 256.0 * scale;

    double center_tx = lon_to_tile_x(center_lon_, z_int);
    double center_ty = lat_to_tile_y(center_lat_, z_int);

    int max_tile = (1 << z_int) - 1;
    int tx_min = std::max(0, static_cast<int>(std::floor(center_tx - (avail.x * 0.5) / tile_w)) - 1);
    int tx_max = std::min(max_tile, static_cast<int>(std::ceil(center_tx + (avail.x * 0.5) / tile_w)) + 1);
    int ty_min = std::max(0, static_cast<int>(std::floor(center_ty - (avail.y * 0.5) / tile_w)) - 1);
    int ty_max = std::min(max_tile, static_cast<int>(std::ceil(center_ty + (avail.y * 0.5) / tile_w)) + 1);

    for (int ty = ty_min; ty <= ty_max; ++ty) {
        for (int tx = tx_min; tx <= tx_max; ++tx) {
            float sx = p_min.x + avail.x * 0.5f + static_cast<float>((tx - center_tx) * tile_w);
            float sy = p_min.y + avail.y * 0.5f + static_cast<float>((ty - center_ty) * tile_w);
            ImVec2 t_pmin(sx, sy);
            ImVec2 t_pmax(sx + static_cast<float>(tile_w), sy + static_cast<float>(tile_w));

            // Visible bounding box test
            if (t_pmax.x < p_min.x || t_pmin.x > p_max.x || t_pmax.y < p_min.y || t_pmin.y > p_max.y) {
                continue;
            }

            TileKey key{static_cast<int>(current_provider_), z_int, tx, ty};
            GLuint tex = get_or_request_texture(key);

            if (tex != 0) {
                draw_list->AddImage(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(tex)), t_pmin, t_pmax);
            } else {
                // Futuristic subtle dark loading tile placeholder
                draw_list->AddRectFilled(t_pmin, t_pmax, IM_COL32(14, 20, 32, 255));
                draw_list->AddRect(t_pmin, t_pmax, IM_COL32(255, 255, 255, 12), 0.0f, 0, 1.0f);
                char coord_lbl[32];
                std::snprintf(coord_lbl, sizeof(coord_lbl), "Z%d / %d / %d", z_int, tx, ty);
                draw_list->AddText(ImVec2(t_pmin.x + 8.0f, t_pmin.y + 8.0f), IM_COL32(100, 130, 160, 120), coord_lbl);
            }
        }
    }

    // --- Render Tactical Overlays (ADS-B, AIS, Airspace, Channels, Radar Rings) ---
    draw_tactical_contacts(draw_list, p_min, p_max, telem,
                           selected_contact_index, is_air_selected,
                           show_air, show_sea, show_trails, show_vectors,
                           show_airspace, show_channels, show_buoys, show_rings, ui_tick);

    // --- On-Map HUD Watermark & Scale Bar ---
    // Scale indicator
    double center_lat_rad = center_lat_ * kDeg2Rad;
    double meters_per_pixel = 156543.03392 * std::cos(center_lat_rad) / std::pow(2.0, zoom_);
    double nmi_per_pixel = meters_per_pixel / 1852.0;
    float scale_bar_pixels = 100.0f;
    double scale_bar_nmi = scale_bar_pixels * nmi_per_pixel;

    ImVec2 scale_pos(p_min.x + 16.0f, p_max.y - 28.0f);
    draw_list->AddRectFilled(ImVec2(scale_pos.x - 4.0f, scale_pos.y - 16.0f),
                             ImVec2(scale_pos.x + scale_bar_pixels + 8.0f, scale_pos.y + 12.0f),
                             IM_COL32(10, 18, 30, 200), 4.0f);
    draw_list->AddLine(ImVec2(scale_pos.x, scale_pos.y), ImVec2(scale_pos.x + scale_bar_pixels, scale_pos.y),
                       IM_COL32(255, 255, 255, 220), 2.0f);
    draw_list->AddLine(ImVec2(scale_pos.x, scale_pos.y - 4.0f), ImVec2(scale_pos.x, scale_pos.y + 4.0f),
                       IM_COL32(255, 255, 255, 220), 2.0f);
    draw_list->AddLine(ImVec2(scale_pos.x + scale_bar_pixels, scale_pos.y - 4.0f),
                       ImVec2(scale_pos.x + scale_bar_pixels, scale_pos.y + 4.0f),
                       IM_COL32(255, 255, 255, 220), 2.0f);

    char scale_text[48];
    if (scale_bar_nmi >= 1.0) {
        std::snprintf(scale_text, sizeof(scale_text), "%.1f NM / %.1f km", scale_bar_nmi, scale_bar_nmi * 1.852);
    } else {
        std::snprintf(scale_text, sizeof(scale_text), "%.0f m", scale_bar_pixels * meters_per_pixel);
    }
    draw_list->AddText(ImVec2(scale_pos.x + 4.0f, scale_pos.y - 14.0f), IM_COL32(220, 235, 255, 240), scale_text);

    // Map attribution badge
    const char* attr = "© OpenStreetMap / CartoDB / Esri";
    draw_list->AddText(ImVec2(p_max.x - 220.0f, p_max.y - 20.0f), IM_COL32(200, 220, 240, 140), attr);

    draw_list->PopClipRect();
    ImGui::EndChild();
}

// -----------------------------------------------------------------------------
// Tactical Contact Icons & Layer Drawing
// -----------------------------------------------------------------------------
void OsmSlippyMap::draw_aircraft_icon(ImDrawList* draw_list, ImVec2 pos, float heading_deg,
                                      float size, ImU32 col_fill, ImU32 col_outline)
{
    float rad = (heading_deg - 90.0f) * kDeg2Rad;
    float cos_a = std::cos(rad);
    float sin_a = std::sin(rad);

    auto rot = [&](float x, float y) -> ImVec2 {
        return ImVec2(pos.x + (x * cos_a - y * sin_a) * size,
                      pos.y + (x * sin_a + y * cos_a) * size);
    };

    // Sleek aircraft silhouette polygon
    ImVec2 poly[] = {
        rot( 1.0f,  0.0f),   // Nose
        rot( 0.2f,  0.3f),   // Right fuselage
        rot(-0.2f,  1.1f),   // Right wing tip
        rot(-0.4f,  1.0f),
        rot(-0.2f,  0.25f),  // Right root
        rot(-0.8f,  0.2f),   // Right tail root
        rot(-1.0f,  0.55f),  // Right tail tip
        rot(-1.1f,  0.45f),
        rot(-0.95f, 0.0f),   // Tail center
        rot(-1.1f, -0.45f),  // Left tail tip
        rot(-1.0f, -0.55f),
        rot(-0.8f, -0.2f),   // Left tail root
        rot(-0.2f, -0.25f),  // Left root
        rot(-0.4f, -1.0f),
        rot(-0.2f, -1.1f),   // Left wing tip
        rot( 0.2f, -0.3f)    // Left fuselage
    };

    draw_list->AddConvexPolyFilled(poly, 16, col_fill);
    draw_list->AddPolyline(poly, 16, col_outline, ImDrawFlags_Closed, 1.2f);
}

void OsmSlippyMap::draw_vessel_icon(ImDrawList* draw_list, ImVec2 pos, float heading_deg,
                                    float size, ImU32 col_fill, ImU32 col_outline)
{
    float rad = (heading_deg - 90.0f) * kDeg2Rad;
    float cos_a = std::cos(rad);
    float sin_a = std::sin(rad);

    auto rot = [&](float x, float y) -> ImVec2 {
        return ImVec2(pos.x + (x * cos_a - y * sin_a) * size,
                      pos.y + (x * sin_a + y * cos_a) * size);
    };

    // Sharp ship hull polygon
    ImVec2 hull[] = {
        rot( 1.1f,  0.0f),   // Bow
        rot( 0.3f,  0.45f),  // Starboard forward
        rot(-0.9f,  0.45f),  // Starboard stern
        rot(-1.0f,  0.0f),   // Stern center
        rot(-0.9f, -0.45f),  // Port stern
        rot( 0.3f, -0.45f)   // Port forward
    };

    draw_list->AddConvexPolyFilled(hull, 6, col_fill);
    draw_list->AddPolyline(hull, 6, col_outline, ImDrawFlags_Closed, 1.2f);
}

void OsmSlippyMap::draw_tactical_contacts(ImDrawList* draw_list, ImVec2 p_min, ImVec2 p_max,
                                         const ExtendedDomainTelemetry& telem,
                                         int& selected_contact_index, bool& is_air_selected,
                                         bool show_air, bool show_sea, bool show_trails,
                                         bool show_vectors, bool show_airspace, bool show_channels,
                                         bool show_buoys, bool show_rings, uint32_t ui_tick)
{
    ImGuiIO& io = ImGui::GetIO();
    bool mouse_clicked = ImGui::IsMouseClicked(ImGuiMouseButton_Left) && ImGui::IsWindowHovered();
    ImVec2 mouse_pos = io.MousePos;

    // 1. Tactical Radar Range Rings (Centered on San Francisco Bay Station: 37.77°N, -122.42°W)
    if (show_rings) {
        ImVec2 station_pt = geo_to_screen(37.7749, -122.4194);

        // Radii in Nautical Miles (5nm, 10nm, 20nm, 40nm)
        float radii_nmi[] = {5.0f, 10.0f, 20.0f, 40.0f};
        for (float r_nmi : radii_nmi) {
            double deg_lat = r_nmi / 60.0;
            double top_lat = 37.7749 + deg_lat;
            ImVec2 top_pt = geo_to_screen(top_lat, -122.4194);
            float r_px = std::abs(station_pt.y - top_pt.y);

            draw_list->AddCircle(station_pt, r_px, IM_COL32(56, 189, 248, 60), 64, 1.2f);

            char ring_lbl[32];
            std::snprintf(ring_lbl, sizeof(ring_lbl), "%.0f NM", r_nmi);
            draw_list->AddText(ImVec2(station_pt.x + 4.0f, station_pt.y - r_px + 2.0f),
                               IM_COL32(56, 189, 248, 120), ring_lbl);
        }

        // Station Center Crosshair
        draw_list->AddCircleFilled(station_pt, 4.0f, IM_COL32(56, 189, 248, 220));
        draw_list->AddLine(ImVec2(station_pt.x - 12.0f, station_pt.y), ImVec2(station_pt.x + 12.0f, station_pt.y),
                           IM_COL32(56, 189, 248, 160), 1.0f);
        draw_list->AddLine(ImVec2(station_pt.x, station_pt.y - 12.0f), ImVec2(station_pt.x, station_pt.y + 12.0f),
                           IM_COL32(56, 189, 248, 160), 1.0f);
        draw_list->AddText(ImVec2(station_pt.x + 8.0f, station_pt.y + 4.0f),
                           IM_COL32(56, 189, 248, 200), "C2 SDR STN-1");
    }

    // 2. Airspace Boundaries (SFO Class B, ILS approaches, Military Restricted)
    if (show_airspace) {
        // SFO Class B 10nm Ring
        ImVec2 sfo_center = geo_to_screen(37.619, -122.375);
        ImVec2 sfo_10nm = geo_to_screen(37.619 + (10.0 / 60.0), -122.375);
        float sfo_r = std::abs(sfo_center.y - sfo_10nm.y);
        draw_list->AddCircle(sfo_center, sfo_r, IM_COL32(56, 189, 248, 70), 48, 1.2f);
        draw_list->AddText(ImVec2(sfo_center.x - 30.0f, sfo_center.y + sfo_r + 2.0f),
                           IM_COL32(56, 189, 248, 140), "SFO CLASS B SFC-100");

        // ILS Runway 28L/R Final Approach Corridor
        ImVec2 ils_start = geo_to_screen(37.580, -122.200);
        ImVec2 ils_end = geo_to_screen(37.615, -122.355);
        draw_list->AddLine(ils_start, ils_end, IM_COL32(56, 189, 248, 100), 1.5f);
        draw_list->AddText(ImVec2(ils_start.x - 35.0f, ils_start.y - 12.0f),
                           IM_COL32(56, 189, 248, 130), "ILS RWY 28R/L");

        // Restricted Military Airspace R-2501
        ImVec2 r_pt = geo_to_screen(37.960, -122.720);
        ImVec2 r_edge = geo_to_screen(37.960 + (6.5 / 60.0), -122.720);
        float r_radius = std::abs(r_pt.y - r_edge.y);
        draw_list->AddCircleFilled(r_pt, r_radius, IM_COL32(239, 68, 68, 20));
        draw_list->AddCircle(r_pt, r_radius, IM_COL32(239, 68, 68, 140), 32, 1.2f);
        draw_list->AddText(ImVec2(r_pt.x - 40.0f, r_pt.y - 6.0f), IM_COL32(239, 68, 68, 200), "RESTRICTED R-2501");
    }

    // 3. Shipping Fairways & Navigation Channels
    if (show_channels) {
        ImVec2 ch_points[] = {
            geo_to_screen(37.750, -122.680), // Western Pilot Approach
            geo_to_screen(37.820, -122.475), // Golden Gate Span
            geo_to_screen(37.825, -122.365), // Central Bay Crossing
            geo_to_screen(37.800, -122.330)  // Port of Oakland Fairway
        };
        for (size_t i = 0; i < 3; ++i) {
            draw_list->AddLine(ch_points[i], ch_points[i + 1], IM_COL32(245, 158, 11, 90), 2.0f);
        }
        draw_list->AddText(ImVec2(ch_points[1].x + 10.0f, ch_points[1].y - 15.0f),
                           IM_COL32(245, 158, 11, 180), "TSS SAN FRANCISCO CHANNEL");
    }

    // 4. Navigation Buoys
    if (show_buoys) {
        bool flash = (static_cast<int>(ui_tick / 15) % 2 == 0);

        // Starboard Green Channel Buoys
        ImVec2 b_g1 = geo_to_screen(37.785, -122.580);
        ImVec2 b_g3 = geo_to_screen(37.815, -122.460);
        draw_list->AddCircleFilled(b_g1, 4.0f, IM_COL32(34, 197, 94, flash ? 255 : 80));
        draw_list->AddText(ImVec2(b_g1.x + 6.0f, b_g1.y - 6.0f), IM_COL32(34, 197, 94, 220), "G \"1\"");
        draw_list->AddCircleFilled(b_g3, 4.0f, IM_COL32(34, 197, 94, flash ? 255 : 80));
        draw_list->AddText(ImVec2(b_g3.x + 6.0f, b_g3.y - 6.0f), IM_COL32(34, 197, 94, 220), "G \"3\"");

        // Port Red Channel Buoys
        ImVec2 b_r2 = geo_to_screen(37.795, -122.580);
        ImVec2 b_r4 = geo_to_screen(37.825, -122.460);
        draw_list->AddCircleFilled(b_r2, 4.0f, IM_COL32(239, 68, 68, !flash ? 255 : 80));
        draw_list->AddText(ImVec2(b_r2.x + 6.0f, b_r2.y - 6.0f), IM_COL32(239, 68, 68, 220), "R \"2\"");
        draw_list->AddCircleFilled(b_r4, 4.0f, IM_COL32(239, 68, 68, !flash ? 255 : 80));
        draw_list->AddText(ImVec2(b_r4.x + 6.0f, b_r4.y - 6.0f), IM_COL32(239, 68, 68, 220), "R \"4\"");
    }

    // 4. AIS Maritime Vessels
    if (show_sea) {
        int sea_count = telem.sea_contact_count > 0 ? telem.sea_contact_count : 5;
        for (int i = 0; i < sea_count; ++i) {
            const auto& s = telem.sea_contacts[i];
            ImVec2 pt = geo_to_screen(s.lat, s.lon);

            // Bounding box cull
            if (pt.x < p_min.x - 50.0f || pt.x > p_max.x + 50.0f ||
                pt.y < p_min.y - 50.0f || pt.y > p_max.y + 50.0f) {
                continue;
            }

            bool is_sel = (!is_air_selected && selected_contact_index == i);

            // Click contact selection detection
            float dist_sq = (mouse_pos.x - pt.x) * (mouse_pos.x - pt.x) + (mouse_pos.y - pt.y) * (mouse_pos.y - pt.y);
            if (mouse_clicked && dist_sq < 256.0f) { // within 16px
                selected_contact_index = i;
                is_air_selected = false;
            }

            // Breadcrumb Wake History
            if (show_trails && s.wake_count > 0) {
                ImVec2 prev_w = pt;
                for (int k = 0; k < s.wake_count && k < 8; ++k) {
                    ImVec2 w_pt = geo_to_screen(s.wake_lat[k], s.wake_lon[k]);
                    draw_list->AddLine(prev_w, w_pt, IM_COL32(14, 165, 233, 140 - k * 16), 1.5f);
                    draw_list->AddCircleFilled(w_pt, 2.0f, IM_COL32(14, 165, 233, 160 - k * 18));
                    prev_w = w_pt;
                }
            }

            // Vessel Colors based on AIS Class
            ImU32 fill_col = IM_COL32(14, 165, 233, 220); // Default cyan
            ImU32 out_col  = IM_COL32(255, 255, 255, 255);
            if (s.vessel_type == 1) fill_col = IM_COL32(245, 158, 11, 230); // Amber (Tanker)
            else if (s.vessel_type == 2) fill_col = IM_COL32(239, 68, 68, 230); // Red (Law/Military)
            else if (s.vessel_type == 3) fill_col = IM_COL32(34, 197, 94, 230);  // Green (Tug/Pilot)

            if (is_sel) {
                fill_col = IM_COL32(234, 179, 8, 255);
                draw_list->AddCircle(pt, 16.0f, IM_COL32(234, 179, 8, 220), 16, 1.8f);
            }

            // Velocity Heading Vector
            if (show_vectors && s.speed_kts > 0.5f) {
                float rad = (s.heading_deg - 90.0f) * kDeg2Rad;
                float vec_len = std::clamp(s.speed_kts * 1.5f, 15.0f, 45.0f);
                ImVec2 vec_end(pt.x + std::cos(rad) * vec_len, pt.y + std::sin(rad) * vec_len);
                draw_list->AddLine(pt, vec_end, fill_col, 1.5f);
            }

            // Vessel Hull Icon
            draw_vessel_icon(draw_list, pt, s.heading_deg, 9.0f, fill_col, out_col);

            // Vessel HUD Data Tag
            char tag[64];
            std::snprintf(tag, sizeof(tag), "%s (%.1f kt)", s.name, s.speed_kts);
            draw_list->AddText(ImVec2(pt.x + 12.0f, pt.y - 12.0f), IM_COL32(224, 242, 254, 230), tag);
        }
    }

    // 5. ADS-B Air Contacts
    if (show_air) {
        int air_count = telem.air_contact_count > 0 ? telem.air_contact_count : 6;
        for (int i = 0; i < air_count; ++i) {
            const auto& a = telem.air_contacts[i];
            ImVec2 pt = geo_to_screen(a.lat, a.lon);

            // Bounding box cull
            if (pt.x < p_min.x - 50.0f || pt.x > p_max.x + 50.0f ||
                pt.y < p_min.y - 50.0f || pt.y > p_max.y + 50.0f) {
                continue;
            }

            bool is_sel = (is_air_selected && selected_contact_index == i);

            // Click contact selection detection
            float dist_sq = (mouse_pos.x - pt.x) * (mouse_pos.x - pt.x) + (mouse_pos.y - pt.y) * (mouse_pos.y - pt.y);
            if (mouse_clicked && dist_sq < 256.0f) { // within 16px
                selected_contact_index = i;
                is_air_selected = true;
            }

            // Breadcrumb History Trail
            if (show_trails && a.trail_count > 0) {
                ImVec2 prev_t = pt;
                for (int k = 0; k < a.trail_count && k < 8; ++k) {
                    ImVec2 t_pt = geo_to_screen(a.trail_lat[k], a.trail_lon[k]);
                    draw_list->AddLine(prev_t, t_pt, IM_COL32(249, 115, 22, 140 - k * 16), 1.5f);
                    draw_list->AddCircleFilled(t_pt, 2.0f, IM_COL32(249, 115, 22, 160 - k * 18));
                    prev_t = t_pt;
                }
            }

            ImU32 fill_col = IM_COL32(249, 115, 22, 230); // Vibrant orange
            ImU32 out_col  = IM_COL32(255, 255, 255, 255);

            if (is_sel) {
                fill_col = IM_COL32(56, 189, 248, 255);
                draw_list->AddCircle(pt, 18.0f, IM_COL32(56, 189, 248, 240), 20, 2.0f);
                draw_list->AddRect(ImVec2(pt.x - 20.0f, pt.y - 20.0f), ImVec2(pt.x + 20.0f, pt.y + 20.0f),
                                   IM_COL32(56, 189, 248, 140), 0.0f, 0, 1.2f);
            }

            // Velocity Vector Line
            if (show_vectors && a.speed_kts > 10.0f) {
                float rad = (a.heading_deg - 90.0f) * kDeg2Rad;
                float vec_len = std::clamp(a.speed_kts * 0.12f, 20.0f, 60.0f);
                ImVec2 vec_end(pt.x + std::cos(rad) * vec_len, pt.y + std::sin(rad) * vec_len);
                draw_list->AddLine(pt, vec_end, fill_col, 1.8f);
            }

            // Aircraft Silhouette Icon
            draw_aircraft_icon(draw_list, pt, a.heading_deg, 11.0f, fill_col, out_col);

            // Aircraft HUD Callout Tag
            char tag1[32], tag2[32];
            std::snprintf(tag1, sizeof(tag1), "%s", a.callsign);
            std::snprintf(tag2, sizeof(tag2), "FL%03.0f %03.0fkt", a.alt_ft / 100.0f, a.speed_kts);

            draw_list->AddText(ImVec2(pt.x + 14.0f, pt.y - 16.0f), IM_COL32(255, 237, 213, 240), tag1);
            draw_list->AddText(ImVec2(pt.x + 14.0f, pt.y - 2.0f), IM_COL32(253, 186, 116, 210), tag2);
        }
    }
}

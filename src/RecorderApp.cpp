/**
 * @file RecorderApp.cpp
 * @brief Recorder -- a take from the mics, the line-in or the pads, trimmed
 *        front and back to a loop.
 *
 *   top    back, the input (MIC / LINE / PADS, a click cycles), REC/STOP,
 *          the take's time, FILES (the takes on the card)
 *   meter  left and right peaks of the input, always on
 *   wave   the take: the loop between the START and END markers lit, the
 *          rest dimmed, and the playhead while it is auditioned
 *   tools  START and END are turned: the first click moves the marker a
 *          column of the picture per detent, the second a millisecond with
 *          the picture zoomed in round it, the third is done. LOOP plays the
 *          marked loop round and round, BARS says what tempo it is as 1, 2,
 *          4 or 8 bars, SAVE writes the loop as a file of its own, SLICE
 *          takes it to the Slicer.
 *
 * Everything that touches the card or the codecs goes through
 * crosspad::ISampleTools; this file only asks and draws. The pads keep
 * playing the kit -- that is what PADS records.
 */

#include "crosspad-recorder/RecorderApp.hpp"

#if __has_include("pc_stubs/PcApp.hpp")
  #include "pc_stubs/PcApp.hpp"
#else
  #include "crosspad-gui/app/App.hpp"
#endif

#include "crosspad/app/AppRegistrar.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/sampletools/ISampleTools.hpp"
#include "crosspad/sampletools/SliceMath.hpp"
#include "crosspad-gui/components/app_orchestrator.h"
#include "crosspad-gui/widgets/gui_helpers.h"

#include "lvgl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifdef ESP_PLATFORM
extern "C" {
  extern lv_indev_t* indev_encoderTop;
}
#define REC_USE_ENCODER 1
#endif

namespace {

constexpr uint16_t kCols       = 300;     // columns of the picture
constexpr uint32_t kTickMs     = 50;
constexpr uint32_t kZoomFrames = 12000;   // each side of a marker in fine mode, ~0.25 s
constexpr uint32_t kReloopMs   = 150;     // a marker still for this long restarts the loop

const lv_color_t kBg     = lv_color_hex(0x16181B);
const lv_color_t kPanel  = lv_color_hex(0x20242A);
const lv_color_t kText   = lv_color_hex(0xE8EAED);
const lv_color_t kDim    = lv_color_hex(0x7B828B);
const lv_color_t kInk    = lv_color_hex(0x16181B);
const lv_color_t kAccent = lv_color_hex(0xC6FF47);
const lv_color_t kRec    = lv_color_hex(0xFF5C5C);
const lv_color_t kAmber  = lv_color_hex(0xFFC857);
const lv_color_t kLoopBg = lv_color_hex(0x263019);
const lv_color_t kWave   = lv_color_hex(0x9FD63A);
const lv_color_t kWaveOff= lv_color_hex(0x4A515B);
const lv_color_t kHead   = lv_color_hex(0xFFFFFF);
const lv_color_t kStartC = lv_color_hex(0x4FE0B0);

/// How a marker is being turned.
enum class Edit : uint8_t { None, Coarse, Fine };

lv_obj_t* s_root = nullptr;
lv_obj_t* s_container = nullptr;       // what the Slicer is started in
lv_obj_t* s_wave = nullptr;
lv_obj_t* s_inBtn = nullptr;
lv_obj_t* s_recBtn = nullptr;
lv_obj_t* s_timeLabel = nullptr;
lv_obj_t* s_info = nullptr;
lv_obj_t* s_meter[2] = {};
lv_obj_t* s_startBtn = nullptr;
lv_obj_t* s_endBtn = nullptr;
lv_obj_t* s_loopBtn = nullptr;
lv_obj_t* s_barsBtn = nullptr;
lv_obj_t* s_overlay = nullptr;
lv_timer_t* s_timer = nullptr;

std::string s_file;                    // the take on screen
uint32_t s_frames = 0;
uint32_t s_rate = 48000;
uint32_t s_start = 0, s_end = 0;       // the loop
crosspad::WaveOverview s_ov;           // the picture drawn
bool     s_ovPending = false;
bool     s_viewWanted = false;         // a picture asked for while a job ran
uint32_t s_wantA = 0, s_wantB = 0;
bool     s_loading = false;            // markers go to the ends when the picture comes
Edit     s_editStart = Edit::None, s_editEnd = Edit::None;
uint8_t  s_bars = 1;
bool     s_looping = false;
bool     s_reloop = false;
uint32_t s_movedAt = 0;
bool     s_wasRecording = false;
int32_t  s_headX = -1;                 // playhead column as drawn, -1 = none
bool     s_listPending = false;
bool     s_savePending = false;
float    s_level[2] = {};

#ifdef REC_USE_ENCODER
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_overlayGroup = nullptr;
#endif

crosspad::ISampleTools* tools() { return crosspad::getPlatformServices().sampleTools; }

std::string base_of(const std::string& p)
{
    const size_t s = p.find_last_of('/');
    return s == std::string::npos ? p : p.substr(s + 1);
}

void set_text(lv_obj_t* l, const char* t)
{
    if (l && std::strcmp(lv_label_get_text(l), t) != 0) lv_label_set_text(l, t);
}

void set_bg(lv_obj_t* o, lv_color_t c)
{
    if (o && !lv_color_eq(lv_obj_get_style_bg_color(o, 0), c)) lv_obj_set_style_bg_color(o, c, 0);
}

// ── the picture ────────────────────────────────────────────────────────────

void request_view(uint32_t a, uint32_t b)
{
    auto* st = tools();
    if (!st || s_file.empty()) return;
    if (st->requestOverview(s_file, a, b, kCols)) {
        s_ovPending = true;
        s_viewWanted = false;
    } else {
        s_viewWanted = true;           // a job is running; asked again from the timer
        s_wantA = a;
        s_wantB = b;
    }
}

void load_file(const std::string& path)
{
    auto* st = tools();
    if (st) st->stopPreview();
    s_looping = false;
    s_file = path;
    s_frames = 0;
    s_start = s_end = 0;
    s_ov = crosspad::WaveOverview{};
    s_loading = true;
    s_editStart = s_editEnd = Edit::None;
    request_view(0, 0);
    if (s_wave) lv_obj_invalidate(s_wave);
}

void fill(lv_layer_t* layer, int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t c)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = c;
    d.bg_opa = LV_OPA_COVER;
    lv_area_t a = {x1, y1, x2, y2};
    lv_draw_rect(layer, &d, &a);
}

void wave_draw_cb(lv_event_t* e)
{
    lv_layer_t* layer = lv_event_get_layer(e);
    lv_obj_t* obj = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (!layer || s_ov.lo.empty() || s_ov.end <= s_ov.start) return;
    lv_area_t box;
    lv_obj_get_content_coords(obj, &box);
    const int32_t w = lv_area_get_width(&box);
    const int32_t h = lv_area_get_height(&box);
    const int32_t mid = box.y1 + h / 2;
    const uint32_t a = s_ov.start, len = s_ov.end - s_ov.start;
    auto xOf = [&](uint32_t f) -> int32_t {
        if (f <= a) return box.x1;
        if (f >= s_ov.end) return box.x2;
        return box.x1 + (int32_t)((uint64_t)(f - a) * w / len);
    };
    /* The loop lit, what is outside it dimmed. */
    const int32_t ls = xOf(s_start), le = xOf(s_end);
    if (le > ls) fill(layer, ls, box.y1, le, box.y2, kLoopBg);
    const size_t cols = s_ov.lo.size();
    for (size_t c = 0; c < cols; ++c) {
        const int32_t x = box.x1 + (int32_t)(c * w / cols);
        const uint32_t f = a + (uint32_t)((uint64_t)c * len / cols);
        const int32_t y1 = mid - (int32_t)s_ov.hi[c] * (h / 2) / 128;
        const int32_t y2 = mid - (int32_t)s_ov.lo[c] * (h / 2) / 128;
        fill(layer, x, std::min(y1, y2), x, std::max(y1, y2), f >= s_start && f < s_end ? kWave : kWaveOff);
    }
    if (s_start >= a && s_start <= s_ov.end) fill(layer, ls, box.y1, ls + 1, box.y2, kStartC);
    if (s_end >= a && s_end <= s_ov.end) fill(layer, le - 1, box.y1, le, box.y2, kRec);
    if (s_headX >= 0) fill(layer, s_headX, box.y1, s_headX, box.y2, kHead);
}

/// The playhead's screen x in the picture, -1 when nothing plays or it is
/// off the picture. Only its old and new columns are redrawn while it
/// moves: the whole picture every tick kept the LVGL task busy enough to
/// starve everything below it on its core.
int32_t head_x()
{
    auto* st = tools();
    if (!st || !s_wave || !st->previewing() || s_ov.end <= s_ov.start) return -1;
    const uint32_t p = st->previewFrame();
    if (p < s_ov.start || p > s_ov.end) return -1;
    lv_area_t box;
    lv_obj_get_content_coords(s_wave, &box);
    return box.x1 + (int32_t)((uint64_t)(p - s_ov.start) * lv_area_get_width(&box) / (s_ov.end - s_ov.start));
}


void move_head()
{
    const int32_t x = head_x();
    if (x == s_headX || !s_wave) return;
    lv_area_t box;
    lv_obj_get_content_coords(s_wave, &box);
    for (int32_t at : {s_headX, x}) {
        if (at < 0) continue;
        lv_area_t a = {at - 1, box.y1, at + 1, box.y2};
        lv_obj_invalidate_area(s_wave, &a);
    }
    s_headX = x;
}

// ── what the screen says ───────────────────────────────────────────────────

void show_info()
{
    if (!s_info) return;
    char buf[112];
    if (s_file.empty()) {
        std::snprintf(buf, sizeof(buf), "Pick an input and press REC -- or FILES for a take on the card.");
    } else if (!s_frames) {
        std::snprintf(buf, sizeof(buf), "%s  reading ...", base_of(s_file).c_str());
    } else {
        const uint32_t loop = s_end > s_start ? s_end - s_start : 0;
        const float bpm = crosspad::SliceMath::bpmOf(loop, s_rate, s_bars);
        std::snprintf(buf, sizeof(buf), "%s  %.2f s | loop %.3f s = %.1f BPM as %u bar%s",
                      base_of(s_file).c_str(), (double)s_frames / s_rate, (double)loop / s_rate,
                      (double)bpm, (unsigned)s_bars, s_bars > 1 ? "s" : "");
    }
    set_text(s_info, buf);
}

void show_marker_labels()
{
    auto label = [](lv_obj_t* b, const char* name, Edit e) {
        if (!b) return;
        char buf[16];
        if (e == Edit::None) std::snprintf(buf, sizeof(buf), "%s", name);
        else std::snprintf(buf, sizeof(buf), "%c %s", name[0], e == Edit::Coarse ? "<>" : "fine");
        set_text(lv_obj_get_child(b, 0), buf);
    };
    label(s_startBtn, "START", s_editStart);
    label(s_endBtn, "END", s_editEnd);
}

void show_transport()
{
    auto* st = tools();
    if (!st) return;
    const crosspad::RecStatus rs = st->recStatus();
    set_text(lv_obj_get_child(s_recBtn, 0), rs.recording ? LV_SYMBOL_STOP " STOP" : "REC");
    set_bg(s_recBtn, rs.recording ? kRec : kPanel);
    static const char* kInputs[] = {"MIC", "LINE", "PADS"};
    set_text(lv_obj_get_child(s_inBtn, 0), kInputs[(int)st->input()]);
    set_bg(s_loopBtn, s_looping ? kAccent : kPanel);
    lv_obj_set_style_text_color(lv_obj_get_child(s_loopBtn, 0), s_looping ? kInk : kText, 0);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%ub", (unsigned)s_bars);
    set_text(lv_obj_get_child(s_barsBtn, 0), buf);
}

// ── markers ────────────────────────────────────────────────────────────────

/// A marker moved: the picture follows it in fine mode, and a loop that is
/// playing is restarted once the marker has stopped moving.
void marker_moved(uint32_t at, Edit e)
{
    if (e == Edit::Fine && (at < s_ov.start + kZoomFrames / 4 || at > s_ov.end - kZoomFrames / 4)) {
        request_view(at > kZoomFrames ? at - kZoomFrames : 0, std::min(s_frames, at + kZoomFrames));
    }
    s_reloop = s_looping;
    s_movedAt = lv_tick_get();
    show_info();
    if (s_wave) lv_obj_invalidate(s_wave);
}

void marker_cb(lv_event_t* e)
{
    const bool isStart = lv_event_get_user_data(e) != nullptr;
    Edit& edit = isStart ? s_editStart : s_editEnd;
    uint32_t& m = isStart ? s_start : s_end;
    lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
    lv_group_t* g = lv_obj_get_group(b);
    const lv_event_code_t code = lv_event_get_code(e);
    if (!s_frames) return;
    if (code == LV_EVENT_FOCUSED) {
        /* The encoder's first click puts the button in edit mode; LVGL says
         * so with FOCUSED, not CLICKED. */
        const bool editing = g && lv_group_get_editing(g);
        if (editing && edit == Edit::None) edit = Edit::Coarse;
        else if (!editing && edit != Edit::None) {
            if (edit == Edit::Fine) request_view(0, 0);
            edit = Edit::None;
        }
        show_marker_labels();
        return;
    }
    if (code == LV_EVENT_CLICKED) {
        if (edit == Edit::Coarse) {
            edit = Edit::Fine;
            request_view(m > kZoomFrames ? m - kZoomFrames : 0, std::min(s_frames, m + kZoomFrames));
            if (g) lv_group_set_editing(g, true);
        } else if (edit == Edit::Fine) {
            edit = Edit::None;
            request_view(0, 0);
            if (g) lv_group_set_editing(g, false);
        } else {
            edit = Edit::Coarse;       // a tap: turn it with the encoder now
            if (g) { lv_group_focus_obj(b); lv_group_set_editing(g, true); }
        }
        show_marker_labels();
        return;
    }
    const uint32_t key = lv_event_get_key(e);
    const int dir = key == LV_KEY_RIGHT || key == LV_KEY_UP ? 1 : (key == LV_KEY_LEFT || key == LV_KEY_DOWN ? -1 : 0);
    if (!dir || edit == Edit::None) return;
    const uint32_t viewLen = s_ov.end > s_ov.start ? s_ov.end - s_ov.start : s_frames;
    const uint32_t step = edit == Edit::Coarse ? std::max<uint32_t>(1, viewLen / kCols)
                                               : std::max<uint32_t>(1, s_rate / 1000);
    int64_t v = (int64_t)m + (int64_t)dir * step;
    const int64_t lo = isStart ? 0 : (int64_t)s_start + 1;
    const int64_t hi = isStart ? (int64_t)s_end - 1 : (int64_t)s_frames;
    v = std::max(lo, std::min(hi, v));
    m = (uint32_t)v;
    marker_moved(m, edit);
}

// ── actions ────────────────────────────────────────────────────────────────

void back_cb(lv_event_t* e)
{
    auto* app = static_cast<App*>(lv_event_get_user_data(e));
    if (app) app->destroyAppAsync();
}

void input_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st) return;
    const int next = ((int)st->input() + 1) % 3;
    st->selectInput((crosspad::RecInput)next);
    show_transport();
}

void rec_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st) return;
    if (st->recStatus().recording) {
        st->stopRecording();
    } else {
        st->stopPreview();
        s_looping = false;
        if (!st->startRecording() && s_info) set_text(s_info, "Cannot record now.");
    }
    show_transport();
}

void loop_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st || !s_frames) return;
    if (s_looping) { st->stopPreview(); s_looping = false; }
    else           s_looping = st->preview(s_file, s_start, s_end, true);
    show_transport();
}

void bars_cb(lv_event_t*)
{
    s_bars = s_bars >= 8 ? 1 : s_bars * 2;
    show_transport();
    show_info();
}

void save_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st || !s_frames) return;
    if (st->requestSaveRegion(s_file, s_start, s_end)) {
        s_savePending = true;
        set_text(s_info, "Saving the loop ...");
    }
}

/// The loop to the Slicer: handed over, then the Slicer opened in this
/// app's place -- on the next turn of the loop, since opening it destroys
/// this screen and its callback with it.
void slice_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st || !s_frames) return;
    st->setHandoff(s_file, s_start, s_end);
    lv_async_call([](void*) {
        auto& orch = crosspad_gui::AppOrchestrator::getInstance();
        for (const auto& app : orch.getApps()) {
            if (app && std::strcmp(app->getName(), "Slicer") == 0) {
                crosspad_gui::AppOrchestrator::onAppSelected(app.get(), s_container);
                return;
            }
        }
    }, nullptr);
}

// ── the takes on the card ──────────────────────────────────────────────────

void close_overlay()
{
    if (!s_overlay) return;
#ifdef REC_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_overlayGroup);     // the click that closed it is still being read
    s_overlayGroup = nullptr;
#endif
    lv_obj_delete_async(s_overlay);
    s_overlay = nullptr;
}

std::vector<std::string> s_list;

void pick_cb(lv_event_t* e)
{
    const int i = (int)(intptr_t)lv_event_get_user_data(e);
    close_overlay();
    if (i >= 0 && i < (int)s_list.size()) load_file(s_list[i]);
    show_info();
}

lv_obj_t* overlay_item(const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(s_overlay);
    lv_obj_set_size(b, LV_PCT(100), 24);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
    lv_obj_set_width(l, LV_PCT(100));
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef REC_USE_ENCODER
    if (s_overlayGroup) lv_group_add_obj(s_overlayGroup, b);
#endif
    return b;
}

void fill_list()
{
    auto* st = tools();
    if (!st) return;
    s_list = st->recordings();
    s_overlay = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_overlay);
    lv_obj_set_size(s_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_overlay, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_overlay, kBg, 0);
    lv_obj_set_style_bg_opa(s_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_overlay, 6, 0);
    lv_obj_set_style_pad_row(s_overlay, 3, 0);
    lv_obj_set_flex_flow(s_overlay, LV_FLEX_FLOW_COLUMN);
#ifdef REC_USE_ENCODER
    s_overlayGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_overlayGroup);
#endif
    lv_obj_t* t = lv_label_create(s_overlay);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_label_set_text(t, s_list.empty() ? "No takes on the card yet." : "Takes on the card");
    for (size_t i = 0; i < s_list.size() && i < 40; ++i) {
        overlay_item(base_of(s_list[i]).c_str(), pick_cb, (void*)(intptr_t)i);
    }
    overlay_item(LV_SYMBOL_CLOSE " Cancel", [](lv_event_t*) { close_overlay(); }, nullptr);
}

void files_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st || s_overlay) return;
    if (st->requestRecordings()) {
        s_listPending = true;
        set_text(s_info, "Reading the card ...");
    }
}

bool back_handler(lv_obj_t*)
{
    if (s_overlay) { close_overlay(); return true; }
    if (s_editStart != Edit::None || s_editEnd != Edit::None) {
#ifdef REC_USE_ENCODER
        if (s_encGroup) lv_group_set_editing(s_encGroup, false);
#endif
        if (s_editStart == Edit::Fine || s_editEnd == Edit::Fine) request_view(0, 0);
        s_editStart = s_editEnd = Edit::None;
        show_marker_labels();
        return true;
    }
    return false;
}

// ── the timer ──────────────────────────────────────────────────────────────

void tick_cb(lv_timer_t*)
{
    auto* st = tools();
    if (!st) return;

    /* Meter: peaks, falling back slowly, as a desk's do. */
    float pk[2];
    st->peaks(pk[0], pk[1]);
    for (int c = 0; c < 2; ++c) {
        s_level[c] = std::max(pk[c], s_level[c] * 0.8f);
        const int32_t pct = std::min<int32_t>(100, (int32_t)(s_level[c] * 100.0f));
        if (s_meter[c]) {
            lv_bar_set_value(s_meter[c], pct, LV_ANIM_OFF);
            const lv_color_t col = pct > 95 ? kRec : (pct > 70 ? kAmber : kAccent);
            if (!lv_color_eq(lv_obj_get_style_bg_color(s_meter[c], LV_PART_INDICATOR), col)) {
                lv_obj_set_style_bg_color(s_meter[c], col, LV_PART_INDICATOR);
            }
        }
    }

    /* The take: its time while it runs, and it on screen when it ends. */
    const crosspad::RecStatus rs = st->recStatus();
    if (rs.recording) {
        char buf[16];
        const uint32_t t10 = rs.sampleRate ? (uint32_t)((uint64_t)rs.frames * 10 / rs.sampleRate) : 0;
        std::snprintf(buf, sizeof(buf), "%02u:%02u.%u", (unsigned)(t10 / 600), (unsigned)(t10 / 10 % 60),
                      (unsigned)(t10 % 10));
        set_text(s_timeLabel, buf);
    }
    if (s_wasRecording && !rs.recording) {
        if (rs.error.empty() && rs.frames) {
            s_rate = rs.sampleRate;
            load_file(rs.file);
        } else {
            char buf[80];
            std::snprintf(buf, sizeof(buf), "The take failed: %s", rs.error.c_str());
            set_text(s_info, buf);
        }
    }
    if (s_wasRecording != rs.recording) show_transport();
    s_wasRecording = rs.recording;

    /* File jobs. */
    const crosspad::ToolJob job = st->job();
    if (job != crosspad::ToolJob::Busy) {
        if (s_ovPending) {
            s_ovPending = false;
            if (job == crosspad::ToolJob::Ready) {
                crosspad::WaveOverview ov = st->overview();
                if (ov.file == s_file) {
                    s_ov = std::move(ov);
                    s_frames = s_ov.frames;
                    if (s_ov.sampleRate) s_rate = s_ov.sampleRate;
                    if (s_loading) { s_start = 0; s_end = s_frames; s_loading = false; }
                }
            } else {
                set_text(s_info, st->jobResult().c_str());
            }
            show_info();
            if (s_wave) lv_obj_invalidate(s_wave);
        } else if (s_listPending) {
            s_listPending = false;
            show_info();
            if (job == crosspad::ToolJob::Ready) fill_list();
        } else if (s_savePending) {
            s_savePending = false;
            char buf[96];
            std::snprintf(buf, sizeof(buf), job == crosspad::ToolJob::Ready ? "Saved the loop as %s" : "Not saved: %s",
                          base_of(st->jobResult()).c_str());
            set_text(s_info, buf);
        } else if (s_viewWanted) {
            request_view(s_wantA, s_wantB);
        }
    }

    /* A moved marker restarts the loop once it has stopped moving. */
    if (s_reloop && lv_tick_elaps(s_movedAt) >= kReloopMs) {
        s_reloop = false;
        if (s_looping) st->preview(s_file, s_start, s_end, true);
    }
    if (s_looping && !st->previewing()) { s_looping = false; show_transport(); }
    move_head();
}

// ── building ───────────────────────────────────────────────────────────────

void style_button(lv_obj_t* b, int32_t w, int32_t h)
{
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, kAccent, LV_STATE_EDITED);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
}

lv_obj_t* add_label(lv_obj_t* b, const char* text)
{
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_obj_set_style_text_color(l, kInk, LV_STATE_EDITED);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return l;
}

lv_obj_t* make_button(lv_obj_t* parent, int32_t w, const char* text, lv_event_cb_t cb, void* ud = nullptr)
{
    lv_obj_t* b = lv_button_create(parent);
    style_button(b, w, 22);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    add_label(b, text);
#ifdef REC_USE_ENCODER
    if (s_encGroup) lv_group_add_obj(s_encGroup, b);
#endif
    return b;
}

lv_obj_t* make_marker(lv_obj_t* parent, const char* text, bool isStart)
{
    lv_obj_t* b = lv_CreateEditableButton(parent);
    style_button(b, 56, 22);
    void* ud = isStart ? (void*)1 : nullptr;
    lv_obj_add_event_cb(b, marker_cb, LV_EVENT_KEY, ud);
    lv_obj_add_event_cb(b, marker_cb, LV_EVENT_CLICKED, ud);
    lv_obj_add_event_cb(b, marker_cb, LV_EVENT_FOCUSED, ud);
    add_label(b, text);
#ifdef REC_USE_ENCODER
    if (s_encGroup) lv_group_add_obj(s_encGroup, b);
#endif
    return b;
}

lv_obj_t* make_row(lv_obj_t* parent, int32_t h)
{
    lv_obj_t* r = lv_obj_create(parent);
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, LV_PCT(100), h);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(r, 3, 0);
    lv_obj_remove_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

lv_obj_t* make_meter(lv_obj_t* parent)
{
    lv_obj_t* m = lv_bar_create(parent);
    lv_obj_set_size(m, LV_PCT(100), 4);
    lv_bar_set_range(m, 0, 100);
    lv_obj_set_style_radius(m, 2, 0);
    lv_obj_set_style_radius(m, 2, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(m, kPanel, 0);
    lv_obj_set_style_bg_opa(m, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(m, kAccent, LV_PART_INDICATOR);
    lv_obj_set_style_anim_duration(m, 0, 0);
    return m;
}

} // namespace

lv_obj_t* Recorder_create(lv_obj_t* parent, App* app)
{
#ifdef REC_USE_ENCODER
    if (indev_encoderTop) s_prevEncGroup = lv_indev_get_group(indev_encoderTop);
    if (!s_encGroup) s_encGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif
    s_container = parent;

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, kBg, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 5, 0);
    lv_obj_set_style_pad_row(s_root, 4, 0);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    // -- input, take, time --
    lv_obj_t* top = make_row(s_root, 22);
    make_button(top, 30, LV_SYMBOL_LEFT, back_cb, app);
    s_inBtn = make_button(top, 48, "MIC", input_cb);
    s_recBtn = make_button(top, 64, "REC", rec_cb);
    s_timeLabel = lv_label_create(top);
    lv_obj_set_flex_grow(s_timeLabel, 1);
    lv_obj_set_style_text_color(s_timeLabel, kText, 0);
    lv_obj_set_style_text_align(s_timeLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_timeLabel, "00:00.0");
    make_button(top, 50, LV_SYMBOL_DIRECTORY " FILES", files_cb);

    // -- meter --
    s_meter[0] = make_meter(s_root);
    s_meter[1] = make_meter(s_root);

    // -- the take --
    s_wave = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_wave);
    lv_obj_set_width(s_wave, LV_PCT(100));
    lv_obj_set_flex_grow(s_wave, 1);
    lv_obj_set_style_bg_color(s_wave, kPanel, 0);
    lv_obj_set_style_bg_opa(s_wave, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_wave, 6, 0);
    lv_obj_set_style_pad_all(s_wave, 3, 0);
    lv_obj_remove_flag(s_wave, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_wave, wave_draw_cb, LV_EVENT_DRAW_MAIN_END, nullptr);

    s_info = lv_label_create(s_root);
    lv_obj_set_style_text_font(s_info, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_info, kDim, 0);
    lv_label_set_long_mode(s_info, LV_LABEL_LONG_DOT);
    lv_obj_set_size(s_info, LV_PCT(100), 12);     // one line: a long one ends in dots, the picture keeps its height
    lv_label_set_text(s_info, "");

    // -- the loop --
    lv_obj_t* tools_row = make_row(s_root, 24);
    s_startBtn = make_marker(tools_row, "START", true);
    s_endBtn = make_marker(tools_row, "END", false);
    s_loopBtn = make_button(tools_row, 48, LV_SYMBOL_LOOP " LOOP", loop_cb);
    s_barsBtn = make_button(tools_row, 32, "1b", bars_cb);
    make_button(tools_row, 44, LV_SYMBOL_SAVE " SAVE", save_cb);
    make_button(tools_row, 52, "SLICE " LV_SYMBOL_RIGHT, slice_cb);

    auto* st = tools();
    if (st) st->selectInput(st->input());   // the meter runs from the start
    s_wasRecording = false;
    show_transport();
    show_marker_labels();
    show_info();

#ifdef REC_USE_ENCODER
    if (s_encGroup) lv_group_focus_obj(s_recBtn);
#endif
    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

    if (app) {
        app->setOnBack(back_handler);
        app->setOnShow([](lv_obj_t*) {
            if (s_timer) lv_timer_resume(s_timer);
#ifdef REC_USE_ENCODER
            if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif
        });
        app->setOnHide([](lv_obj_t*) {
            if (s_timer) lv_timer_pause(s_timer);
#ifdef REC_USE_ENCODER
            if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
#endif
        });
    }
    return s_root;
}

void Recorder_destroy(lv_obj_t* root)
{
    if (s_timer) {
        lv_timer_delete(s_timer);
        s_timer = nullptr;
    }
    /* A take still running is finished and kept; the codec input goes back
     * to what the capture preset had. */
    if (auto* st = tools()) st->releaseInput();
#ifdef REC_USE_ENCODER
    if (s_overlayGroup) { lv_group_delete(s_overlayGroup); s_overlayGroup = nullptr; }
#endif
    s_overlay = nullptr;
    s_root = s_wave = s_inBtn = s_recBtn = s_timeLabel = s_info = nullptr;
    s_meter[0] = s_meter[1] = nullptr;
    s_startBtn = s_endBtn = s_loopBtn = s_barsBtn = nullptr;
    s_looping = s_reloop = s_ovPending = s_viewWanted = s_listPending = s_savePending = false;
    s_editStart = s_editEnd = Edit::None;
    s_headX = -1;

#ifdef REC_USE_ENCODER
    if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
    if (s_encGroup) {
        lv_group_delete(s_encGroup);
        s_encGroup = nullptr;
    }
    s_prevEncGroup = nullptr;
#endif
    lv_obj_delete_async(root);
}

// No pad logic: the pads play the kit, which is what a PADS take records.
REGISTER_APP(Recorder, nullptr, "recorder.png",
             Recorder_create, Recorder_destroy,
             nullptr, nullptr, nullptr, 0)

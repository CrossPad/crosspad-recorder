/**
 * @file RecorderApp.cpp
 * @brief Recorder -- a take from the mics, the line-in or the pads, trimmed
 *        front and back to a loop.
 *
 *   top    back, the input (MIC / LINE / PADS / USB -- what the computer
 *          plays over USB audio; a click cycles), REC/STOP,
 *          the take's time, FILES (any WAV on the card, a folder at a time)
 *   song   SYNC: REC plays the open song from bar FROM and the take lines up
 *          with it -- its frame 0 is that bar, the codecs' delay taken off --
 *          so a verse is recorded over the beat; a song already playing is
 *          not restarted, the take goes in at its next bar line; sent here by
 *          Arrange, SYNC is on and FROM is where its playhead stood; the time is then the song's
 *          bar.beat. PRE counts one or two bars in first, on the metronome's
 *          click, and the time counts the beats down. -> ARR copies the loop between the markers into the
 *          song's clips and lays it on a track (a synced take at the bar it
 *          was sung on, the START marker's part of a bar kept as silence).
 *          CLIP EDIT on an Arrange audio block opens that block here: its
 *          WAV, the markers on the part of it the block plays; trimmed and
 *          put back, it takes the block's place, every frame where it was.
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
#include "crosspad/instrument/IInstrumentHost.hpp"
#include "crosspad/instrument/InstrumentPadLogic.hpp"
#include "crosspad/instrument/InstrumentPlayState.hpp"
#include "crosspad/pad/PadManager.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/sampletools/ISampleTools.hpp"
#include "crosspad/settings/CrosspadSettings.hpp"
#include "crosspad/settings/ISettingsUI.hpp"
#include "crosspad/sampletools/SliceMath.hpp"
#include "crosspad-gui/components/mode_overlay.h"
#include "crosspad/song/ArrangeFocus.hpp"
#include "crosspad/song/ISongEngine.hpp"
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
constexpr uint32_t kProgressMs = 300;     // a redraw this slow gets a progress bar; a new take gets one at once

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
lv_obj_t* s_progress = nullptr;        // over the picture while it is being drawn
lv_obj_t* s_progressBar = nullptr;
lv_obj_t* s_progressLabel = nullptr;
lv_obj_t* s_meter[2] = {};
lv_obj_t* s_startBtn = nullptr;
lv_obj_t* s_endBtn = nullptr;
lv_obj_t* s_loopBtn = nullptr;
lv_obj_t* s_barsBtn = nullptr;
lv_obj_t* s_overlay = nullptr;
lv_timer_t* s_timer = nullptr;
lv_obj_t* s_syncBtn = nullptr;
lv_obj_t* s_preBtn = nullptr;
lv_obj_t* s_fromBtn = nullptr;
lv_obj_t* s_songLabel = nullptr;
lv_obj_t* s_trackBtn = nullptr;        // in the To Arrange window
lv_obj_t* s_barBtn = nullptr;
lv_obj_t* s_arrInfo = nullptr;
lv_obj_t* s_setInfo = nullptr;         // the Settings window answers here, not behind itself

std::string s_file;                    // the take on screen
uint32_t s_frames = 0;
uint32_t s_rate = 48000;
uint32_t s_start = 0, s_end = 0;       // the loop
crosspad::WaveOverview s_ov;           // the picture drawn
bool     s_ovPending = false;
uint32_t s_ovAskedAt = 0;
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

/* Where the markers land on a new take. Persisted, not kept in this file: a
 * player ticks it once and expects every take after it to arrive trimmed, a
 * reboot included (CrosspadSettings::recorderAutoTrim). */
bool auto_trim_on()
{
    auto* cfg = crosspad::CrosspadSettings::getInstance();
    return cfg && cfg->recorderAutoTrim;
}

void auto_trim_set(bool on)
{
    auto* cfg = crosspad::CrosspadSettings::getInstance();
    if (!cfg || cfg->recorderAutoTrim == on) return;
    cfg->recorderAutoTrim = on;
    /* Written now rather than when the screen closes: the board can be
     * switched off from the transport at any time. */
    if (auto* ui = crosspad::getSettingsUI()) ui->saveSettings();
}

/* With the song: kept while the board runs, as the Sequencer keeps its tempo. */
bool     s_sync = false;               // REC plays the song and the take lines up with it
uint32_t s_from = 0;                   // the bar it plays from, 0-based
bool     s_fromTouched = false;        // until turned, FROM follows the song's loop region
uint8_t  s_preBars = 0;                // count-in before a take with the song: 0, 1 or 2 bars
bool     s_takeWithSong = false;       // the take on screen: its frame s_takeOrigin is bar s_takeFrom
uint32_t s_takeFrom = 0;
uint32_t s_takeOrigin = 0;             // 0 for a take; an Arrange block's offset into its file
/* An Arrange block opened by CLIP EDIT: where the markers go once the picture
 * comes (the part the block plays, 0 = to the file's end), and which block it
 * is, so PUT can take it out before its trimmed copy goes in. */
bool     s_block = false;
uint32_t s_blockA = 0, s_blockB = 0;
crosspad::SongClip s_blockClip;
uint8_t  s_blockTrack = 0;
uint8_t  s_arrTrack = 0;               // To Arrange: where the loop goes
uint32_t s_arrBar = 0;
uint32_t s_arrLead = 0;                // silence before it, the START marker's part of a bar
bool     s_importPending = false;

#ifdef REC_USE_ENCODER
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_overlayGroup = nullptr;
#endif

crosspad::ISampleTools* tools() { return crosspad::getPlatformServices().sampleTools; }
crosspad::ISongEngine* songs() { return crosspad::getPlatformServices().songEngine; }

/// The open song, as far as this screen needs it.
struct SongView {
    bool        open = false;
    std::string name;
    int         bpm = 0;
    uint8_t     beatsPerBar = 4;
    uint32_t    end = 0;               // bars it plays over, region included; 0 = nothing to play
    uint32_t    regionStart = 0;
    bool        region = false;
    uint32_t    spBar = 0;             // at the take's rate
};

SongView song_view()
{
    SongView v;
    auto* se = songs();
    if (!se) return v;
    std::lock_guard<std::recursive_mutex> l(se->songMutex());
    const crosspad::AudioSong& s = se->song();      // the first look opens the last project
    v.open = se->openState() == crosspad::SongOpState::Ready;
    v.name = s.name;
    v.bpm = (int)lroundf(s.bpm > 0.0f ? s.bpm : crosspad::AudioSong::kDefaultBpm);
    v.beatsPerBar = s.beatsPerBar;
    v.region = s.hasRegion();
    v.regionStart = s.loopStart;
    v.end = std::max(s.playEnd(), v.region ? s.loopEnd : 0u);
    v.spBar = s.samplesPerBar(s_rate);
    return v;
}

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
    if (o && !lv_color_eq(lv_obj_get_style_bg_color(o, LV_PART_MAIN), c)) lv_obj_set_style_bg_color(o, c, 0);
}

// ── the picture ────────────────────────────────────────────────────────────

void request_view(uint32_t a, uint32_t b)
{
    auto* st = tools();
    if (!st || s_file.empty()) return;
    if (st->requestOverview(s_file, a, b, kCols)) {
        s_ovPending = true;
        s_ovAskedAt = lv_tick_get();
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

/// Put the markers where the take sounds. Reads the envelope that came with
/// the picture, so it costs no card: the whole take is one overview, and its
/// hop is ~5 ms, which is the accuracy a transient is found to before the
/// guard either side is added.
bool apply_auto_trim()
{
    if (!s_frames || s_ov.envelope.empty()) return false;
    const crosspad::SliceMath::TrimPoints t =
        crosspad::SliceMath::autoTrim(s_ov.envelope, s_ov.hop, s_ov.start, s_frames, s_rate);
    s_start = t.start;
    s_end   = t.end;
    return t.start != 0 || t.end != s_frames;
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
        if (s_block) {
            std::snprintf(buf, sizeof(buf), "T%u block on bar %u | %s | marked %.2f s | ARR puts it back",
                          (unsigned)(s_blockTrack + 1), (unsigned)(s_blockClip.bar + 1),
                          base_of(s_file).c_str(), (double)loop / s_rate);
        } else if (s_takeWithSong) {
            std::snprintf(buf, sizeof(buf), "%s  %.2f s | sung over the song from bar %u | marked %.2f s",
                          base_of(s_file).c_str(), (double)s_frames / s_rate, (unsigned)(s_takeFrom + 1),
                          (double)loop / s_rate);
        } else {
            std::snprintf(buf, sizeof(buf), "%s  %.2f s | loop %.3f s = %.1f BPM as %u bar%s",
                          base_of(s_file).c_str(), (double)s_frames / s_rate, (double)loop / s_rate,
                          (double)bpm, (unsigned)s_bars, s_bars > 1 ? "s" : "");
        }
    }
    set_text(s_info, buf);
}

/// A take of minutes is seconds of card to draw: the picture says how far it
/// is. A new take shows it at once, a redraw only if it is slow.
void show_progress(crosspad::ISampleTools* st)
{
    if (!s_progress) return;
    const bool waiting = s_ovPending || s_viewWanted;
    if (!waiting || !(s_loading || lv_tick_elaps(s_ovAskedAt) >= kProgressMs)) {
        if (!lv_obj_has_flag(s_progress, LV_OBJ_FLAG_HIDDEN)) lv_obj_add_flag(s_progress, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    const uint32_t pct = s_ovPending && st->job() == crosspad::ToolJob::Busy ? st->jobProgress() : 0;
    if (lv_bar_get_value(s_progressBar) != (int32_t)pct) lv_bar_set_value(s_progressBar, (int32_t)pct, LV_ANIM_OFF);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "Drawing the take ... %u%%", (unsigned)pct);
    set_text(s_progressLabel, buf);
    if (lv_obj_has_flag(s_progress, LV_OBJ_FLAG_HIDDEN)) lv_obj_remove_flag(s_progress, LV_OBJ_FLAG_HIDDEN);
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
    static const char* kInputs[] = {"MIC", "LINE", "PADS", "USB", "PITCH"};
    set_text(lv_obj_get_child(s_inBtn, 0), kInputs[(int)st->input()]);
    set_bg(s_loopBtn, s_looping ? kAccent : kPanel);
    lv_obj_set_style_text_color(lv_obj_get_child(s_loopBtn, 0), s_looping ? kInk : kText, 0);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%ub", (unsigned)s_bars);
    set_text(lv_obj_get_child(s_barsBtn, 0), buf);
    set_bg(s_syncBtn, s_sync ? kAccent : kPanel);
    lv_obj_set_style_text_color(lv_obj_get_child(s_syncBtn, 0), s_sync ? kInk : kText, 0);
    std::snprintf(buf, sizeof(buf), "FROM %u", (unsigned)(s_from + 1));
    set_text(lv_obj_get_child(s_fromBtn, 0), buf);
    if (s_preBars) std::snprintf(buf, sizeof(buf), "PRE %u", (unsigned)s_preBars);
    else           std::snprintf(buf, sizeof(buf), "PRE");
    set_text(lv_obj_get_child(s_preBtn, 0), buf);
    set_bg(s_preBtn, s_preBars ? kAccent : kPanel);
    lv_obj_set_style_text_color(lv_obj_get_child(s_preBtn, 0), s_preBars ? kInk : kText, 0);
}

/// The song row: which song SYNC plays, at what tempo, how long.
void show_song()
{
    if (!s_songLabel) return;
    const SongView v = song_view();
    char buf[64];
    if (!songs())      std::snprintf(buf, sizeof(buf), "no songs on this board");
    else if (!v.open)  std::snprintf(buf, sizeof(buf), "opening the song ...");
    else if (!v.end)   std::snprintf(buf, sizeof(buf), "%s: empty", v.name.c_str());
    else               std::snprintf(buf, sizeof(buf), "%s  %d BPM", v.name.c_str(), v.bpm);
    set_text(s_songLabel, buf);
    /* FROM follows the loop region until it is turned by hand. */
    if (!s_fromTouched && v.open) s_from = v.region ? v.regionStart : 0;
    if (v.end && s_from >= v.end) s_from = v.end - 1;
}

/// A number turned with the encoder: a click (or a tap) to turn it, another
/// to leave. Returns the step a turn asks for, 0 for anything else.
int number_step(lv_event_t* e)
{
    lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
    lv_group_t* g = b ? lv_obj_get_group(b) : nullptr;
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (g) {
            const bool editing = lv_group_get_editing(g) && lv_group_get_focused(g) == b;
            lv_group_focus_obj(b);
            lv_group_set_editing(g, !editing);
        }
        return 0;
    }
    const uint32_t key = lv_event_get_key(e);
    return key == LV_KEY_RIGHT || key == LV_KEY_UP ? 1 : (key == LV_KEY_LEFT || key == LV_KEY_DOWN ? -1 : 0);
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

crosspad::IInstrumentHost* inst_host() { return crosspad::getPlatformServices().instrumentHost; }

/* PITCH: the pads play the instrument Pitch Play has, and the take is that,
 * digital, off the same bus PADS records. There is no instrument picker here
 * on purpose -- one place sets the instrument and the key it is played in,
 * and that place is Pitch Play (core's InstrumentPlayState). This borrows the
 * engine for whatever is set there and gives it back on the way out. */
bool s_pitchHeld = false;          ///< this app took the engine and owes it back
bool s_pitchReady = false;         ///< the bank is installed and the pads sound
bool s_padsHeld = false;           ///< the pad logic is pushed

/* The pads making the notes is core's behaviour, the same object the
 * Sequencer pushes for its own instrument source: one place decides what a
 * pad plays, and it is InstrumentPlayState. */
constexpr const char* kPitchPadLogic = "RecPitch";
std::shared_ptr<crosspad::InstrumentPadLogic> s_pitchPadLogic;

void pitch_drop_pads()
{
    if (!s_padsHeld) return;
    crosspad::getPadManager().popPadLogic();
    s_padsHeld = false;
}

/// Before Ready, so a hit in the gap is silent rather than a kit pad into a
/// take that is meant to be the instrument.
void pitch_take_pads()
{
    if (s_padsHeld) return;
    auto& pm = crosspad::getPadManager();
    if (!s_pitchPadLogic) {
        s_pitchPadLogic = std::make_shared<crosspad::InstrumentPadLogic>();
        pm.registerPadLogic(kPitchPadLogic, s_pitchPadLogic);
    }
    pm.pushPadLogic(kPitchPadLogic);
    s_padsHeld = true;
}

void pitch_release()
{
    pitch_drop_pads();
    if (s_pitchPadLogic) s_pitchPadLogic->setReady(false);
    s_pitchReady = false;
    if (!s_pitchHeld) return;
    s_pitchHeld = false;
    if (auto* h = inst_host()) h->release();   // the current kit loads again
}

void pitch_take()
{
    auto* h = inst_host();
    const auto& ps = crosspad::instrumentPlayState();
    if (!h) { set_text(s_info, "No instrument engine on this build."); return; }
    if (ps.instrumentPath.empty()) {
        set_text(s_info, "Open an instrument in Pitch Play first -- PITCH records that one.");
        return;
    }
    if (!h->requestOpen(ps.instrumentPath)) { set_text(s_info, "The engine is busy -- try again."); return; }
    s_pitchHeld = true;
    s_pitchReady = false;
    pitch_take_pads();
    set_text(s_info, ("Loading " + ps.instrumentName + " ...").c_str());
}

/// From the app tick: the load runs on the host's worker, so the pads are
/// opened only once it reports the bank is playing.
void pitch_poll()
{
    if (!s_pitchHeld) return;
    auto* h = inst_host();
    if (!h) return;
    /* Pitch Play may have been opened and the key changed while this app sat
     * behind it; the pads read the map live, so only the gate is polled. */
    switch (h->state()) {
    case crosspad::IInstrumentHost::State::Ready:
        if (!s_pitchReady) {
            s_pitchReady = true;
            if (s_pitchPadLogic) s_pitchPadLogic->setReady(true);
            const std::string msg = h->message();
            set_text(s_info, msg.empty() ? "The pads play it -- REC records what they play." : msg.c_str());
        }
        break;
    case crosspad::IInstrumentHost::State::Failed: {
        const std::string why = h->message();
        pitch_release();
        set_text(s_info, why.empty() ? "Could not load that instrument." : why.c_str());
        break;
    }
    default:
        break;
    }
}

void input_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st) return;
    const int cur  = (int)st->input();
    const int next = (cur + 1) % 5;
    if (cur == (int)crosspad::RecInput::Pitch) pitch_release();
    st->selectInput((crosspad::RecInput)next);
    if (next == (int)crosspad::RecInput::Pitch) {
        pitch_take();                 // says itself why, when it cannot
    } else if (!st->inputAvailable((crosspad::RecInput)next)) {
        set_text(s_info, "USB records what the computer plays -- switch Settings -> USB to audio.");
    }
    show_transport();
}

/// With SYNC, the song plays from FROM and the take starts with it -- or,
/// with the song already playing (the transport bar's PLAY), goes in at its
/// next bar line; STOP stops both.
void rec_cb(lv_event_t*)
{
    auto* st = tools();
    if (!st) return;
    if (st->recStatus().recording) {
        st->stopRecording();
    } else if (s_sync) {
        const SongView v = song_view();
        if (!v.open)      set_text(s_info, "The song is not open yet -- is the card in?");
        else if (!v.end)  set_text(s_info, "SYNC plays the open song, and it is empty: make a beat in Arrange first.");
        else {
            st->stopPreview();
            s_looping = false;
            const uint8_t beats = (uint8_t)(s_preBars * v.beatsPerBar);
            if (!st->startRecordingWithSong(std::min(s_from, v.end - 1), beats)) set_text(s_info, "Cannot record now.");
        }
    } else {
        st->stopPreview();
        s_looping = false;
        if (!st->startRecording() && s_info) set_text(s_info, "Cannot record now.");
    }
    show_transport();
}

void sync_cb(lv_event_t*)
{
    auto* st = tools();
    if (st && st->recStatus().recording) return;
    s_sync = !s_sync;
    show_song();
    show_transport();
}

/// Count-in before a take with the song: off, a bar, two bars.
void pre_cb(lv_event_t*)
{
    s_preBars = (uint8_t)((s_preBars + 1) % 3);
    if (s_preBars && !s_sync && s_info) set_text(s_info, "PRE counts in before the song: turn SYNC on.");
    show_transport();
}

void from_cb(lv_event_t* e)
{
    const int step = number_step(e);
    if (!step) return;
    const SongView v = song_view();
    const int64_t hi = v.end ? (int64_t)v.end - 1 : 0;
    s_from = (uint32_t)std::max<int64_t>(0, std::min<int64_t>(hi, (int64_t)s_from + step));
    s_fromTouched = true;
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
    s_trackBtn = s_barBtn = s_arrInfo = s_setInfo = nullptr;
}

/* The card, a folder at a time: FILES opens the recordings, or the folder
 * last looked in; the rows are paged, forty objects rather than a folder's
 * worth. */
constexpr uint32_t kPageRows = 40;
std::string          s_browseDir;      // "" = the recordings, until a folder is opened
crosspad::CardFolder s_folder;
uint32_t             s_page = 0;

void browse(const std::string& dir)
{
    auto* st = tools();
    if (!st) return;
    close_overlay();
    if (st->requestFolder(dir)) {
        s_listPending = true;
        set_text(s_info, "Reading the card ...");
    } else {
        set_text(s_info, "The card is busy -- try again.");
    }
}

void fill_list();

void entry_cb(lv_event_t* e)
{
    const size_t i = (size_t)(intptr_t)lv_event_get_user_data(e);
    if (i >= s_folder.size()) return;
    const std::string path = s_folder.path + "/" + s_folder.name(i);
    if (s_folder.isFolder(i)) { browse(path); return; }
    close_overlay();
    s_takeWithSong = false;            // a file off the card is not known to line up with anything
    s_takeOrigin = 0;
    s_block = false;
    load_file(path);
    show_info();
}

/// Focus as a frame inside the button: the theme's ring stands 4 px out of it,
/// and the rows these buttons fill cut it to brackets.
void focus_frame(lv_obj_t* b)
{
    lv_obj_set_style_outline_width(b, 0, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_border_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_border_width(b, 2, LV_STATE_FOCUS_KEY);
}

lv_obj_t* overlay_item(const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(s_overlay);
    lv_obj_set_size(b, LV_PCT(100), 24);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    focus_frame(b);
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

void overlay_note(const char* text)
{
    lv_obj_t* l = lv_label_create(s_overlay);
    lv_obj_set_style_text_color(l, kDim, 0);
    lv_label_set_text(l, text);
}

void fill_list()
{
    auto* st = tools();
    if (!st) return;
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
    const std::string root = st->cardRoot();
    const bool atRoot = s_folder.path.size() <= root.size();
    std::string title = atRoot ? std::string("/") : s_folder.path.substr(root.size());
    if (title.size() > 36) title = "..." + title.substr(title.size() - 33);
    lv_obj_t* t = lv_label_create(s_overlay);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_label_set_text(t, title.c_str());
    if (!atRoot) {
        overlay_item(LV_SYMBOL_UP " ..", [](lv_event_t*) {
            browse(s_folder.path.substr(0, s_folder.path.find_last_of('/')));
        }, nullptr);
    }
    if (s_page) {
        overlay_item(LV_SYMBOL_UP " earlier", [](lv_event_t*) {
            s_page -= std::min(s_page, kPageRows);
            close_overlay();
            fill_list();
        }, nullptr);
    }
    const size_t end = std::min<size_t>(s_folder.size(), (size_t)s_page + kPageRows);
    for (size_t i = s_page; i < end; ++i) {
        const std::string label = std::string(s_folder.isFolder(i) ? LV_SYMBOL_DIRECTORY " " : LV_SYMBOL_AUDIO " ") +
                                  s_folder.name(i);
        overlay_item(label.c_str(), entry_cb, (void*)(intptr_t)i);
    }
    if (end < s_folder.size()) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), LV_SYMBOL_DOWN " %u more", (unsigned)(s_folder.size() - end));
        overlay_item(buf, [](lv_event_t*) {
            s_page += kPageRows;
            close_overlay();
            fill_list();
        }, nullptr);
    }
    if (!s_folder.size()) overlay_note("No folders or WAVs here.");
    if (s_folder.truncated) overlay_note("Only the first 2000 are listed.");
    overlay_item(LV_SYMBOL_CLOSE " Cancel", [](lv_event_t*) { close_overlay(); }, nullptr);
}

void files_cb(lv_event_t*)
{
    if (s_overlay) return;
    browse(s_browseDir);
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
    pitch_poll();

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
        auto* se = songs();
        const uint8_t count = rs.withSong && rs.waiting && se ? se->countInBeatsLeft() : 0;
        if (count) {
            std::snprintf(buf, sizeof(buf), "count %u", (unsigned)count);
        } else if (rs.withSong && rs.waiting) {
            std::snprintf(buf, sizeof(buf), "wait ...");
        } else if (rs.withSong && se) {
            /* Where the song is, as bar.beat: what a verse is counted in. */
            const double pos = se->positionBars();
            const uint32_t bar = (uint32_t)pos;
            const uint32_t beat = (uint32_t)((pos - bar) * song_view().beatsPerBar);
            std::snprintf(buf, sizeof(buf), "bar %u.%u", (unsigned)(bar + 1), (unsigned)(beat + 1));
        } else {
            const uint32_t t10 = rs.sampleRate ? (uint32_t)((uint64_t)rs.frames * 10 / rs.sampleRate) : 0;
            std::snprintf(buf, sizeof(buf), "%02u:%02u.%u", (unsigned)(t10 / 600), (unsigned)(t10 / 10 % 60),
                          (unsigned)(t10 % 10));
        }
        set_text(s_timeLabel, buf);
    }
    if (s_wasRecording && !rs.recording) {
        if (rs.error.empty() && rs.frames) {
            s_rate = rs.sampleRate;
            s_takeWithSong = rs.withSong;
            s_takeFrom = rs.fromBar;
            s_takeOrigin = 0;
            s_block = false;
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
                    if (s_loading) {
                        s_start = 0;
                        s_end = s_frames;
                        s_loading = false;
                        if (s_block) {
                            /* An Arrange block: on the part of the file it plays. */
                            s_start = std::min(s_blockA, s_frames);
                            s_end = s_blockB ? std::min(s_blockB, s_frames) : s_frames;
                            if (s_end <= s_start) { s_start = 0; s_end = s_frames; }
                        /* The ends first, so a take with nothing to find keeps
                         * them even if the envelope is unreadable. */
                        } else if (auto_trim_on() && apply_auto_trim()) {
                            set_text(s_info, "Auto trim: the markers are on the take.");
                        }
                        show_marker_labels();
                    }
                }
            } else {
                set_text(s_info, st->jobResult().c_str());
            }
            show_info();
            if (s_wave) lv_obj_invalidate(s_wave);
        } else if (s_listPending) {
            s_listPending = false;
            show_info();
            if (job == crosspad::ToolJob::Ready) {
                s_folder = st->folder();
                s_browseDir = s_folder.path;
                s_page = 0;
                fill_list();
            } else {
                set_text(s_info, st->jobResult().c_str());
            }
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
    show_progress(st);

    /* A loop on its way into the song. */
    if (s_importPending) {
        auto* se = songs();
        const crosspad::SongOpState es = se ? se->editState() : crosspad::SongOpState::Failed;
        char buf[112];
        if (es == crosspad::SongOpState::Busy) {
            std::snprintf(buf, sizeof(buf), "Putting it in the song ... %u%%", (unsigned)se->editProgress());
        } else {
            s_importPending = false;
            if (es == crosspad::SongOpState::Ready) {
                std::snprintf(buf, sizeof(buf), "On track %u from bar %u -- open Arrange to hear it with the beat.",
                              (unsigned)(s_arrTrack + 1), (unsigned)(s_arrBar + 1));
                /* Arrange sent us here to fill a track and the take is on it:
                 * the loop closes where it started, and nothing is being
                 * filled any more. */
                crosspad::ArrangeFocus& f = crosspad::arrangeFocus();
                if (f.held() && !f.seq) {
                    f.clear();
                    crosspad_gui::mode_overlay_open_app("Arrange");
                    return;
                }
            } else {
                std::snprintf(buf, sizeof(buf), "Not put in the song: the card did not take it.");
            }
            show_song();
        }
        set_text(s_info, buf);
    }
    /* The song opens after the screen, and can change under it. */
    static uint8_t songTicks = 0;
    if (++songTicks >= 20) { songTicks = 0; show_song(); show_transport(); }

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
    /* On the button, not the label: only the button is ever EDITED, and the
     * label takes the colour from it -- dark on the accent while turned. */
    lv_obj_set_style_text_color(b, kText, 0);
    lv_obj_set_style_text_color(b, kInk, LV_STATE_EDITED);
    focus_frame(b);
}

lv_obj_t* add_label(lv_obj_t* b, const char* text)
{
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
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
    /* 46, not 56: a marker never shows more than "START", and the eight pixels
     * each of them gives up are what PLAY sits in. */
    style_button(b, 46, 22);
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

/// A number the encoder turns (see number_step), in @p g.
lv_obj_t* make_number(lv_obj_t* parent, int32_t w, int32_t h, lv_event_cb_t cb, lv_group_t* g)
{
    lv_obj_t* b = lv_CreateEditableButton(parent);
    style_button(b, w, h);
    lv_obj_add_event_cb(b, cb, LV_EVENT_KEY, nullptr);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* l = add_label(b, "");
    lv_obj_add_flag(l, LV_OBJ_FLAG_EVENT_BUBBLE);
    if (g) lv_group_add_obj(g, b);
    return b;
}

// ── to Arrange ─────────────────────────────────────────────────────────────

/// Bars the loop takes on the song's grid, its lead included.
uint32_t arr_bars(const SongView& v)
{
    const uint32_t len = s_arrLead + (s_end > s_start ? s_end - s_start : 0);
    return v.spBar ? std::max<uint32_t>(1, (len + v.spBar - 1) / v.spBar) : 1;
}

/// What the loop would do to the blocks on its track: nothing, or which go.
void show_arr()
{
    auto* se = songs();
    if (!se || !s_arrInfo) return;
    const SongView v = song_view();
    const uint32_t bars = arr_bars(v), to = s_arrBar + bars;
    int replaced = 0;
    bool cuts = false;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        for (const auto& c : se->song().tracks[s_arrTrack].clips) {
            if (c.bar >= s_arrBar && c.bar < to) ++replaced;
            else if (c.bar < s_arrBar && c.bar + c.bars > s_arrBar) cuts = true;
        }
    }
    char buf[128];
    int n = std::snprintf(buf, sizeof(buf), "Bars %u-%u on track %u", (unsigned)(s_arrBar + 1), (unsigned)to,
                          (unsigned)(s_arrTrack + 1));
    if (!replaced && !cuts) std::snprintf(buf + n, sizeof(buf) - n, ": free.");
    else std::snprintf(buf + n, sizeof(buf) - n, ": %d block%s there go%s.", replaced, replaced == 1 ? "" : "s",
                       cuts ? ", the one before is cut" : "");
    set_text(s_arrInfo, buf);
    char t[16];
    std::snprintf(t, sizeof(t), "TRACK %u", (unsigned)(s_arrTrack + 1));
    set_text(lv_obj_get_child(s_trackBtn, 0), t);
    std::snprintf(t, sizeof(t), "BAR %u", (unsigned)(s_arrBar + 1));
    set_text(lv_obj_get_child(s_barBtn, 0), t);
}

void track_cb(lv_event_t* e)
{
    const int step = number_step(e);
    if (!step) return;
    s_arrTrack = (uint8_t)std::max(0, std::min(crosspad::AudioSong::kTracks - 1, (int)s_arrTrack + step));
    show_arr();
}

void bar_cb(lv_event_t* e)
{
    const int step = number_step(e);
    if (!step) return;
    s_arrBar = (uint32_t)std::max<int64_t>(0, std::min<int64_t>(998, (int64_t)s_arrBar + step));
    show_arr();
}

void put_cb(lv_event_t*)
{
    auto* se = songs();
    if (!se) return;
    close_overlay();
    /* An Arrange block put back on its track takes its own place: the old one
     * goes first, or a START moved later would leave its head playing in
     * front of the trimmed copy. Undo brings it back. */
    if (s_block && s_arrTrack == s_blockTrack) {
        {
            std::lock_guard<std::recursive_mutex> l(se->songMutex());
            auto& cs = se->song().tracks[s_blockTrack].clips;
            for (size_t i = 0; i < cs.size(); ++i) {
                const auto& c = cs[i];
                if (c.bar == s_blockClip.bar && c.file == s_blockClip.file && c.offset == s_blockClip.offset) {
                    cs.erase(cs.begin() + (long)i);
                    break;
                }
            }
        }
        se->commit();
    }
    if (se->requestImport(s_file, s_start, s_end, s_arrTrack, s_arrBar, s_arrLead)) {
        s_importPending = true;
        set_text(s_info, "Putting it in the song ...");
    } else {
        set_text(s_info, "The song is busy -- try again.");
    }
}

lv_obj_t* overlay_row()
{
    lv_obj_t* r = lv_obj_create(s_overlay);
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, LV_PCT(100), 30);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(r, 6, 0);
    lv_obj_remove_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

// ── settings ───────────────────────────────────────────────────────────────

/// Ticking it is a request to see it: the take already on screen is trimmed
/// now rather than at the next one. Unticking leaves the markers where they
/// are -- they may have been turned by hand since -- and only stops the next
/// take being trimmed.
void autotrim_cb(lv_event_t* e)
{
    const bool on = lv_obj_has_state((lv_obj_t*)lv_event_get_target(e), LV_STATE_CHECKED);
    auto_trim_set(on);
    if (on && s_frames) {
        if (auto* st = tools()) st->stopPreview();
        s_looping = false;
        if (apply_auto_trim()) set_text(s_setInfo, "The markers are on the take.");
        else                   set_text(s_setInfo, "Nothing to cut off this take.");
        show_marker_labels();
        show_transport();
        if (s_wave) lv_obj_invalidate(s_wave);
    }
}

/// Hear it as a pad will: once, from START, rather than round and round like
/// LOOP beside it. A one-shot is how this trim is judged -- whether the hit
/// lands the instant it is struck -- and a loop hides exactly that, since the
/// second time round there is a tail sitting in front of the attack.
///
/// @p reportTo is where it answers: the line on the screen, or the Settings
/// window's own when that is covering it.
void play_once(lv_obj_t* reportTo)
{
    auto* st = tools();
    if (!st) return;
    if (!s_frames || s_end <= s_start) {
        set_text(reportTo, "Record a take, or pick one with FILES, first.");
        return;
    }
    st->stopPreview();
    s_looping = false;                 // the loop and the one-shot are not both on
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f s from START", (double)(s_end - s_start) / s_rate);
    set_text(reportTo, st->preview(s_file, s_start, s_end, false) ? buf : "Cannot play now.");
    show_transport();
}

void play_cb(lv_event_t*)          { play_once(s_info); }
void settings_play_cb(lv_event_t*) { play_once(s_setInfo); }

/// What the screen does by itself, as against what a button does. One
/// setting so far; the window is a list so the next one is a row, not a
/// redesign.
void show_settings()
{
    s_overlay = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_overlay);
    lv_obj_set_size(s_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_overlay, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_overlay, kBg, 0);
    lv_obj_set_style_bg_opa(s_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_overlay, 6, 0);
    lv_obj_set_style_pad_row(s_overlay, 5, 0);
    lv_obj_set_flex_flow(s_overlay, LV_FLEX_FLOW_COLUMN);
#ifdef REC_USE_ENCODER
    s_overlayGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_overlayGroup);
#endif
    lv_obj_t* t = lv_label_create(s_overlay);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_label_set_text(t, LV_SYMBOL_SETTINGS " Settings");

    lv_obj_t* cb = lv_checkbox_create(s_overlay);
    lv_checkbox_set_text(cb, "Auto trim");
    lv_obj_set_width(cb, LV_PCT(100));
    lv_obj_set_style_text_color(cb, kText, 0);
    lv_obj_set_style_bg_color(cb, kPanel, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(cb, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_border_color(cb, kDim, LV_PART_INDICATOR);
    lv_obj_set_style_border_width(cb, 1, LV_PART_INDICATOR);
    lv_obj_set_style_radius(cb, 4, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(cb, kAccent, (lv_style_selector_t)LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(cb, kInk, (lv_style_selector_t)LV_PART_INDICATOR | LV_STATE_CHECKED);
    /* The focus ring goes on the box, where the eye is; the theme's outline
     * round the whole row reads as the label being selected. */
    lv_obj_set_style_outline_width(cb, 0, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_border_color(cb, kHead, (lv_style_selector_t)LV_PART_INDICATOR | LV_STATE_FOCUS_KEY);
    lv_obj_set_style_border_width(cb, 2, (lv_style_selector_t)LV_PART_INDICATOR | LV_STATE_FOCUS_KEY);
    if (auto_trim_on()) lv_obj_add_state(cb, LV_STATE_CHECKED);
    lv_obj_add_event_cb(cb, autotrim_cb, LV_EVENT_VALUE_CHANGED, nullptr);
#ifdef REC_USE_ENCODER
    if (s_overlayGroup) lv_group_add_obj(s_overlayGroup, cb);
#endif

    overlay_note("START goes to the take's attack, END to where the sound\n"
                 "stops -- the room before and after is left out of the loop.\n"
                 "A take with no silence to cut keeps its ends.");

    overlay_item(LV_SYMBOL_PLAY " Play it once, as a pad would", settings_play_cb, nullptr);

    s_setInfo = lv_label_create(s_overlay);
    lv_obj_set_style_text_font(s_setInfo, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_setInfo, kDim, 0);
    lv_obj_set_width(s_setInfo, LV_PCT(100));
    lv_label_set_long_mode(s_setInfo, LV_LABEL_LONG_DOT);
    lv_label_set_text(s_setInfo, "");

    overlay_item(LV_SYMBOL_CLOSE " Close", [](lv_event_t*) { close_overlay(); }, nullptr);
#ifdef REC_USE_ENCODER
    if (s_overlayGroup) lv_group_focus_obj(cb);
#endif
}

void settings_cb(lv_event_t*)
{
    if (s_overlay) return;
    show_settings();
}

/// The marked loop into the song: a synced take goes where it was sung --
/// the bar under START, the rest of that bar before START kept as silence
/// -- on the first track with room there; any other take after the last
/// block of the first empty track. Both can be turned before PUT.
void arr_cb(lv_event_t*)
{
    auto* st = tools();
    auto* se = songs();
    if (!st || s_overlay || st->recStatus().recording) return;
    if (!se) { set_text(s_info, "No songs on this board."); return; }
    if (!s_frames || s_end <= s_start) { set_text(s_info, "Record a take, or pick one with FILES, first."); return; }
    const SongView v = song_view();
    if (!v.open) { set_text(s_info, "The song is not open yet -- is the card in?"); return; }
    /* Frame s_takeOrigin of the file sounds on bar s_takeFrom: START lands as
     * many bars and frames from there -- before it, too, for a block whose
     * START was moved earlier than the part it played. */
    const int64_t spb = v.spBar ? (int64_t)v.spBar : 1;
    const int64_t rel = (int64_t)s_start - (int64_t)s_takeOrigin;
    const int64_t shift = rel >= 0 ? rel / spb : -((-rel + spb - 1) / spb);
    s_arrLead = s_takeWithSong && v.spBar ? (uint32_t)(rel - shift * spb) : 0;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        const crosspad::AudioSong& song = se->song();
        const int last = crosspad::AudioSong::kTracks - 1;
        if (s_takeWithSong) {
            const int64_t bar = (int64_t)s_takeFrom + shift;
            s_arrBar = bar > 0 ? (uint32_t)bar : 0;
            if (bar < 0) s_arrLead = 0;   // the song has nothing before its bar one
            const uint32_t to = s_arrBar + arr_bars(v);
            s_arrTrack = (uint8_t)last;
            for (int t = 0; t <= last; ++t) {
                bool free = true;
                for (const auto& c : song.tracks[t].clips) free = free && (c.bar + c.bars <= s_arrBar || c.bar >= to);
                if (free) { s_arrTrack = (uint8_t)t; break; }
            }
        } else {
            s_arrTrack = (uint8_t)last;
            for (int t = 0; t <= last; ++t) {
                if (song.tracks[t].clips.empty()) { s_arrTrack = (uint8_t)t; break; }
            }
            s_arrBar = song.trackEnd(s_arrTrack);
        }
        /* Arrange sent us to fill one particular track: that one, whatever
         * the guess above picked. The player chose AUDIO on "New track" and
         * this take is what goes on it -- putting it anywhere else would
         * leave the track they just made empty. Still turnable before PUT. */
        const crosspad::ArrangeFocus& f = crosspad::arrangeFocus();
        if (f.held() && !f.seq && song.tracks[f.track].kind != crosspad::TrackKind::Seq) {
            s_arrTrack = (uint8_t)f.track;
            if (!s_takeWithSong) s_arrBar = song.trackEnd(s_arrTrack);
        }
    }

    s_overlay = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_overlay);
    lv_obj_set_size(s_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_overlay, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_overlay, kBg, 0);
    lv_obj_set_style_bg_opa(s_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_overlay, 8, 0);
    lv_obj_set_style_pad_row(s_overlay, 8, 0);
    lv_obj_set_flex_flow(s_overlay, LV_FLEX_FLOW_COLUMN);
    lv_group_t* g = nullptr;
#ifdef REC_USE_ENCODER
    s_overlayGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_overlayGroup);
    g = s_overlayGroup;
#endif
    lv_obj_t* t = lv_label_create(s_overlay);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_obj_set_width(t, LV_PCT(100));
    lv_label_set_long_mode(t, LV_LABEL_LONG_DOT);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "To Arrange: %s, %d BPM", v.name.c_str(), v.bpm);
    lv_label_set_text(t, buf);

    lv_obj_t* src = lv_label_create(s_overlay);
    lv_obj_set_style_text_font(src, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(src, kDim, 0);
    std::snprintf(buf, sizeof(buf), "%s, %.2f s from the markers%s", base_of(s_file).c_str(),
                  (double)(s_end - s_start) / s_rate, s_takeWithSong ? ", where it was sung" : "");
    lv_label_set_text(src, buf);

    lv_obj_t* row = overlay_row();
    s_trackBtn = make_number(row, 96, 30, track_cb, g);
    s_barBtn = make_number(row, 96, 30, bar_cb, g);

    s_arrInfo = lv_label_create(s_overlay);
    lv_obj_set_style_text_font(s_arrInfo, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_arrInfo, kText, 0);
    lv_obj_set_width(s_arrInfo, LV_PCT(100));
    lv_label_set_long_mode(s_arrInfo, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_arrInfo, "");

    lv_obj_t* act = overlay_row();
    lv_obj_t* put = make_button(act, 96, LV_SYMBOL_OK " PUT", put_cb, nullptr);
    lv_obj_set_height(put, 30);
    lv_obj_set_style_bg_color(put, kAccent, 0);
    lv_obj_set_style_text_color(lv_obj_get_child(put, 0), kInk, 0);
    lv_obj_t* cancel = make_button(act, 96, LV_SYMBOL_CLOSE " CANCEL", [](lv_event_t*) { close_overlay(); }, nullptr);
    lv_obj_set_height(cancel, 30);
#ifdef REC_USE_ENCODER
    /* make_button() put them in the screen's group; they belong to this one. */
    for (lv_obj_t* b : {put, cancel}) {
        lv_group_remove_obj(b);
        if (g) lv_group_add_obj(g, b);
    }
    if (g) lv_group_focus_obj(put);
#endif
    show_arr();
}

/// CLIP EDIT on an Arrange audio track: the block of it on @p f's bar -- the
/// one picked, or the one under the playhead -- here, its WAV with the
/// markers on the part it plays, lined up with the song where it sits. No
/// block there, and the screen records into the track as before.
void open_block(const crosspad::ArrangeFocus& f)
{
    auto* se = songs();
    if (!se) return;
    std::string path;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        const crosspad::AudioSong& song = se->song();
        const auto& cs = song.tracks[f.track].clips;
        for (size_t i = 0; i < cs.size() && path.empty(); ++i) {
            uint32_t end = cs[i].bar + cs[i].bars;
            if (i + 1 < cs.size()) end = std::min(end, cs[i + 1].bar);
            if (cs[i].file.empty() || f.bar < cs[i].bar || f.bar >= end) continue;
            s_blockClip = cs[i];
            path = song.clipPath(cs[i].file);
        }
    }
    if (path.empty()) return;
    s_block = true;
    s_blockTrack = (uint8_t)f.track;
    s_blockA = s_blockClip.offset;
    s_blockB = s_blockClip.frames ? s_blockClip.offset + s_blockClip.frames : 0;
    s_takeWithSong = true;
    s_takeFrom = s_blockClip.bar;
    s_takeOrigin = s_blockClip.offset;
    s_from = s_blockClip.bar;
    load_file(path);
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

    /* Sent by Arrange to fill an audio track: the take is for that song, from
     * where its playhead stood -- SYNC on, FROM there. */
    {
        const crosspad::ArrangeFocus& f = crosspad::arrangeFocus();
        s_block = false;               // the last visit's block is not this one's
        if (f.held() && !f.seq) {
            s_sync = true;
            s_from = f.bar;
            s_fromTouched = true;
            open_block(f);
        }
    }

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
    make_button(top, 26, LV_SYMBOL_SETTINGS, settings_cb);

    // -- with the song --
    lv_obj_t* songRow = make_row(s_root, 22);
    s_syncBtn = make_button(songRow, 44, "SYNC", sync_cb);
    s_preBtn = make_button(songRow, 40, "PRE", pre_cb);
    lv_group_t* encGroup = nullptr;
#ifdef REC_USE_ENCODER
    encGroup = s_encGroup;
#endif
    s_fromBtn = make_number(songRow, 54, 22, from_cb, encGroup);
    s_songLabel = lv_label_create(songRow);
    lv_obj_set_flex_grow(s_songLabel, 1);
    lv_obj_set_style_text_font(s_songLabel, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_songLabel, kDim, 0);
    lv_label_set_long_mode(s_songLabel, LV_LABEL_LONG_DOT);
    lv_label_set_text(s_songLabel, "");
    make_button(songRow, 50, LV_SYMBOL_RIGHT " ARR", arr_cb);

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

    s_progress = lv_obj_create(s_wave);
    lv_obj_remove_style_all(s_progress);
    lv_obj_set_size(s_progress, LV_PCT(70), LV_SIZE_CONTENT);
    lv_obj_center(s_progress);
    lv_obj_set_flex_flow(s_progress, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_progress, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_progress, 4, 0);
    lv_obj_remove_flag(s_progress, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_progress, LV_OBJ_FLAG_CLICKABLE);
    s_progressLabel = lv_label_create(s_progress);
    lv_obj_set_style_text_font(s_progressLabel, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_progressLabel, kText, 0);
    lv_label_set_text(s_progressLabel, "");
    s_progressBar = lv_bar_create(s_progress);
    lv_obj_set_size(s_progressBar, LV_PCT(100), 6);
    lv_bar_set_range(s_progressBar, 0, 100);
    lv_obj_set_style_radius(s_progressBar, 3, 0);
    lv_obj_set_style_radius(s_progressBar, 3, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_progressBar, kBg, 0);
    lv_obj_set_style_bg_opa(s_progressBar, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_progressBar, kWave, LV_PART_INDICATOR);
    lv_obj_set_style_anim_duration(s_progressBar, 0, 0);
    lv_obj_add_flag(s_progress, LV_OBJ_FLAG_HIDDEN);

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
    make_button(tools_row, 28, LV_SYMBOL_PLAY, play_cb);
    s_loopBtn = make_button(tools_row, 44, LV_SYMBOL_LOOP, loop_cb);
    s_barsBtn = make_button(tools_row, 32, "1b", bars_cb);
    make_button(tools_row, 44, LV_SYMBOL_SAVE " SAVE", save_cb);
    make_button(tools_row, 50, "SLICE " LV_SYMBOL_RIGHT, slice_cb);

    auto* st = tools();
    if (st) st->selectInput(st->input());   // the meter runs from the start
    s_wasRecording = false;
    show_song();
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
    /* And the engine, if PITCH had it: the kit loads again, so the launcher's
     * pads are the kit's. */
    pitch_release();
#ifdef REC_USE_ENCODER
    if (s_overlayGroup) { lv_group_delete(s_overlayGroup); s_overlayGroup = nullptr; }
#endif
    s_overlay = nullptr;
    s_root = s_wave = s_inBtn = s_recBtn = s_timeLabel = s_info = nullptr;
    s_progress = s_progressBar = s_progressLabel = nullptr;
    s_meter[0] = s_meter[1] = nullptr;
    s_startBtn = s_endBtn = s_loopBtn = s_barsBtn = nullptr;
    s_syncBtn = s_preBtn = s_fromBtn = s_songLabel = s_trackBtn = s_barBtn = s_arrInfo = nullptr;
    s_setInfo = nullptr;
    s_looping = s_reloop = s_ovPending = s_viewWanted = s_listPending = s_savePending = s_importPending = false;
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
    if (root) lv_obj_delete_async(root);
}

/* No padLogicName in the registration on purpose: the pads play the kit,
 * which is what a PADS take records, and PITCH pushes core's instrument logic
 * for as long as it holds the engine. */
REGISTER_APP(Recorder, nullptr, "recorder.png",
             Recorder_create, Recorder_destroy,
             nullptr, nullptr, nullptr, 0)

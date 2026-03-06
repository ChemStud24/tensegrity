import asyncio
import json
import threading
import time
from pathlib import Path
from typing import Dict

import cv2
import matplotlib.pyplot as plt
import mujoco
import numpy
import numpy as np
import tqdm
from matplotlib.widgets import Button, Slider

try:
    import aiohttp.web
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False


def _draw_overlay_on_frame(frame, overlay_data, column_width=180):
    """Draw HUD overlay with robot state information on a frame.

    Shared function used by both PassiveViewer and RemoteViewer.

    Args:
        frame: BGR image (numpy array) to draw on
        overlay_data: dict with keys:
            - 'com': (x, y) center of mass in meters
            - 'heading': heading angle in radians
            - 'cable_lengths': list of cable lengths in mm
            - 'rest_lengths': list of rest lengths in mm (first 6 cables)
            - 'controls': list of control values [-1, 1]
        column_width: Width of the overlay column in pixels (default: 180)
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.45
    thickness = 1
    color = (255, 255, 255)  # White text
    bg_color = (0, 0, 0)  # Black background
    line_height = 20
    padding = 8

    frame_height, frame_width = frame.shape[:2]

    lines = []

    # Playback time
    if 'time_text' in overlay_data:
        lines.append(overlay_data['time_text'])
        lines.append("")

    # Center of mass
    if 'com' in overlay_data:
        com = overlay_data['com']
        lines.append(f"CoM X: {com[0]:.3f} m")
        lines.append(f"CoM Y: {com[1]:.3f} m")

    # Heading angle
    if 'heading' in overlay_data:
        heading_deg = np.rad2deg(overlay_data['heading'])
        lines.append(f"Heading: {heading_deg:.1f} deg")

    # Add separator
    if lines:
        lines.append("")

    # Cable lengths (one per line for better readability)
    if 'cable_lengths' in overlay_data:
        lengths = overlay_data['cable_lengths']
        lines.append("Cable Lengths (mm):")
        for i, length in enumerate(lengths):
            lines.append(f"  C{i}: {length:.0f}")

    # Add separator
    if 'cable_lengths' in overlay_data:
        lines.append("")

    # Rest lengths (first 6 cables)
    if 'rest_lengths' in overlay_data:
        rest_lengths = overlay_data['rest_lengths']
        lines.append("Rest Lengths (mm):")
        for i, rest_length in enumerate(rest_lengths):
            lines.append(f"  R{i}: {rest_length:.0f}")

    # Add separator
    if 'rest_lengths' in overlay_data:
        lines.append("")

    # Controls (one per line for better readability)
    if 'controls' in overlay_data:
        controls = overlay_data['controls']
        lines.append("Controls:")
        for i, ctrl in enumerate(controls):
            lines.append(f"  M{i}: {ctrl:+.2f}")

    # Calculate background rectangle dimensions
    bg_height = len(lines) * line_height + 2 * padding
    bg_width = column_width

    # Position on the right side of the frame
    x_start = frame_width - bg_width
    y_start = 0

    # Draw solid background for the entire right column
    cv2.rectangle(frame, (x_start, y_start),
                  (frame_width, bg_height), bg_color, -1)

    # Draw text lines
    y = y_start + padding + line_height - 5
    for line in lines:
        if line == "":  # Skip empty separator lines
            y += line_height // 2
            continue
        cv2.putText(frame, line, (x_start + padding, y), font, font_scale,
                    color, thickness, cv2.LINE_AA)
        y += line_height


_VIEWER_HTML = '''<!DOCTYPE html>
<html><head><title>MuJoCo Remote Viewer</title>
<style>
*{margin:0;padding:0}
body{background:#1a1a1a;overflow:hidden}
canvas{display:block;cursor:grab}
canvas:active{cursor:grabbing}
#hud{position:fixed;top:8px;left:8px;color:#aaa;font:13px monospace;
     background:rgba(0,0,0,.6);padding:4px 8px;border-radius:4px}
#pauseBtn{
  position:fixed;bottom:8px;right:8px;z-index:10;
  color:#fff;background:rgba(0,0,0,.7);border:1px solid #666;
  border-radius:4px;padding:6px 10px;font:13px monospace;cursor:pointer
}
#pauseBtn:hover{background:rgba(255,255,255,.15)}
#restartBtn{
  position:fixed;bottom:8px;right:84px;z-index:10;
  color:#fff;background:rgba(0,0,0,.7);border:1px solid #666;
  border-radius:4px;padding:6px 10px;font:13px monospace;cursor:pointer
}
#restartBtn:hover{background:rgba(255,255,255,.15)}
#pauseBtnHud{
  margin-left:8px;
  color:#fff;background:rgba(0,0,0,.7);border:1px solid #666;
  border-radius:4px;padding:2px 8px;font:12px monospace;cursor:pointer
}
#pauseBtnHud:hover{background:rgba(255,255,255,.15)}
</style></head><body>
<canvas id="v"></canvas><div id="hud">Connecting... <button id="pauseBtnHud">Pause</button></div><button id="restartBtn">Restart</button><button id="pauseBtn">Pause</button>
<script>
const C=document.getElementById('v'),X=C.getContext('2d'),H=document.getElementById('hud');
const P=document.getElementById('pauseBtn');
const R=document.getElementById('restartBtn');
const PH=document.getElementById('pauseBtnHud');
let ws,fc=0,ft=performance.now();
let paused=false;
function setPaused(v){
  paused=v;
  const label=paused?'Resume':'Pause';
  P.textContent=label;
  PH.textContent=label;
}
function sendPauseToggle(){
  if(ws&&ws.readyState===1){
    ws.send(JSON.stringify({t:'ctrl',a:'toggle_pause'}));
  }
}
function sendRestart(){
  if(ws&&ws.readyState===1){
    ws.send(JSON.stringify({t:'ctrl',a:'restart'}));
  }
}
P.addEventListener('click',sendPauseToggle);
R.addEventListener('click',sendRestart);
PH.addEventListener('click',sendPauseToggle);
window.addEventListener('keydown',e=>{
  if(e.code==='Space'){
    e.preventDefault();
    sendPauseToggle();
  }else if(e.code==='KeyR'){
    e.preventDefault();
    sendRestart();
  }
});
function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.binaryType='arraybuffer';
  ws.onopen=()=>{H.textContent='Connected';setPaused(false)};
  ws.onclose=()=>{H.textContent='Reconnecting...';setTimeout(connect,1000)};
  ws.onerror=()=>{};
  ws.onmessage=e=>{
    if(typeof e.data==='string'){
      try{
        const m=JSON.parse(e.data);
        if(m&&m.t==='pause'){setPaused(!!m.paused)}
      }catch(_){}
      return;
    }
    createImageBitmap(new Blob([e.data],{type:'image/jpeg'})).then(b=>{
      if(C.width!==b.width||C.height!==b.height){C.width=b.width;C.height=b.height}
      X.drawImage(b,0,0);b.close();fc++;
      const n=performance.now();
      if(n-ft>1000){H.textContent=Math.round(fc*1000/(n-ft))+' FPS';fc=0;ft=n}
    })
  };
}
connect();
let lx=0,ly=0,btn=[false,false,false];
C.addEventListener('contextmenu',e=>e.preventDefault());
C.addEventListener('mousedown',e=>{e.preventDefault();lx=e.offsetX;ly=e.offsetY;btn[e.button]=true});
window.addEventListener('mouseup',e=>{btn[e.button]=false});
C.addEventListener('mousemove',e=>{
  if(!btn[0]&&!btn[1]&&!btn[2])return;
  const dx=e.offsetX-lx,dy=e.offsetY-ly;lx=e.offsetX;ly=e.offsetY;
  const a=btn[2]?'move':btn[0]?'rotate':'zoom';
  if(ws&&ws.readyState===1)ws.send(JSON.stringify({t:'m',a,dx,dy,h:C.height}))
});
C.addEventListener('wheel',e=>{
  e.preventDefault();
  if(ws&&ws.readyState===1)ws.send(JSON.stringify({t:'s',dy:e.deltaY}))
},{passive:false});
</script></body></html>'''


class RemoteViewer:
    """WebSocket-based remote MuJoCo viewer accessible via browser.

    Renders frames offscreen (EGL), encodes as JPEG, and streams to connected
    browser clients over WebSocket. Works headlessly -- no display required.
    Same API as PassiveViewer: sync(), render(), is_running(), close().
    """

    def __init__(self, model, data, port=8765, width=960, height=720, jpeg_quality=30,
                 overlay_callback=None, max_fps=None):
        if not AIOHTTP_AVAILABLE:
            raise ImportError("aiohttp is required for RemoteViewer. Install with: pip install aiohttp")

        self.model = model
        self.data = data
        self._running = True
        self._dirty = True
        self._port = port
        self.width = width
        self.height = height
        self._overlay_callback = overlay_callback
        self._jpeg_quality = jpeg_quality
        self._target_fps = max_fps or 30

        # Keep for non-threaded render path (PassiveViewer)
        self._min_frame_interval = 1.0 / max_fps if max_fps else 0.0
        self._last_render_time = 0.0

        # Ensure model offscreen framebuffer is large enough
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, height)

        # Thread-safe shared state for async communication
        self._lock = threading.Lock()
        self._mouse_events = []
        self._connected_clients = set()
        self._paused = False
        self._restart_requested = False

        # GL init: on render thread for remote viewers, on main thread for local
        self._render_thread = None
        if port != -1:
            self._gl_ready = threading.Event()
            self._render_thread = threading.Thread(
                target=self._render_loop, name="RemoteViewerRender", daemon=True
            )
            self._render_thread.start()
            self._gl_ready.wait()
        else:
            self._init_gl()

        # aiohttp server in a daemon thread (only if port is specified)
        self._loop = None
        self._server_thread = None
        if port != -1:  # -1 means no server (used by PassiveViewer)
            self._server_thread = threading.Thread(
                target=self._run_server, name="RemoteViewerServer", daemon=True
            )
            self._server_thread.start()

    # -- GL initialization (called from render thread or main thread) ----------

    def _init_gl(self):
        """Initialize OpenGL context and MuJoCo rendering objects."""
        self._gl_ctx = mujoco.gl_context.GLContext(self.width, self.height)
        self._gl_ctx.make_current()

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance = 12.0
        self.cam.elevation = -30
        self.cam.azimuth = 90
        self.opt = mujoco.MjvOption()
        self.pert = mujoco.MjvPerturb()
        self.con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.con)

        self._rgb = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self._viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        self._jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]

    # -- Background render loop (runs on its own thread) ----------------------

    def _render_loop(self):
        """Render loop running independently of the physics thread."""
        self._init_gl()
        self._gl_ready.set()

        while self._running:
            frame_start = time.monotonic()
            self._do_render()
            elapsed = time.monotonic() - frame_start
            sleep_time = (1.0 / self._target_fps) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _do_render(self):
        """Perform one frame of rendering and broadcast to clients."""
        # Drain pending mouse events and apply to camera
        with self._lock:
            events = self._mouse_events
            self._mouse_events = []
        for ev in events:
            self._apply_mouse_event(ev)

        if not self._dirty and not events:
            return

        self._gl_ctx.make_current()

        mujoco.mjv_updateScene(
            self.model, self.data, self.opt, self.pert, self.cam,
            mujoco.mjtCatBit.mjCAT_ALL, self.scene,
        )
        mujoco.mjr_render(self._viewport, self.scene, self.con)
        mujoco.mjr_readPixels(self._rgb, None, self._viewport, self.con)

        # OpenGL renders bottom-up; flip, then BGR for cv2
        frame = np.flipud(self._rgb)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Draw overlay if callback is provided
        if self._overlay_callback is not None:
            try:
                overlay_data = self._overlay_callback()
                self._draw_overlay(frame_bgr, overlay_data)
            except Exception:
                pass  # Silently ignore overlay errors

        ok, buf = cv2.imencode('.jpg', frame_bgr, self._jpeg_params)
        self._dirty = False

        if ok and self._loop is not None and self._loop.is_running():
            jpeg = buf.tobytes()
            asyncio.run_coroutine_threadsafe(self._broadcast(jpeg), self._loop)

    # -- Public API (matches PassiveViewer) ------------------------------------

    def sync(self):
        """Mark data as dirty. Thread-safe; called from physics thread."""
        self._dirty = True

    def render(self):
        """Render frame and broadcast to clients.

        No-op when the background render thread is active (remote viewer).
        Falls back to synchronous rendering for PassiveViewer (port=-1).
        """
        if self._render_thread is not None:
            return  # Render thread handles rendering independently

        if not self._running:
            return

        now = time.monotonic()
        if self._min_frame_interval and now - self._last_render_time < self._min_frame_interval:
            return

        self._do_render()
        self._last_render_time = time.monotonic()

    def is_running(self):
        return self._running

    def close(self):
        self._running = False
        if self._render_thread is not None:
            self._render_thread.join(timeout=2.0)
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)

    # -- aiohttp server -------------------------------------------------------

    def _run_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        app = aiohttp.web.Application()
        app.router.add_get('/', self._handle_index)
        app.router.add_get('/ws', self._handle_ws)

        runner = aiohttp.web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = aiohttp.web.TCPSite(runner, '0.0.0.0', self._port)
        self._loop.run_until_complete(site.start())
        self._loop.run_forever()
        self._loop.run_until_complete(runner.cleanup())

    async def _handle_index(self, request):
        return aiohttp.web.Response(
            text=_VIEWER_HTML,
            content_type='text/html',
            headers={
                'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
                'Pragma': 'no-cache',
                'Expires': '0',
            },
        )

    async def _handle_ws(self, request):
        ws = aiohttp.web.WebSocketResponse()
        await ws.prepare(request)

        with self._lock:
            self._connected_clients.add(ws)
            paused_state = self._paused

        await ws.send_json({'t': 'pause', 'paused': paused_state})

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        ev = json.loads(msg.data)
                        with self._lock:
                            if ev.get('t') == 'ctrl' and ev.get('a') == 'toggle_pause':
                                self._paused = not self._paused
                                paused_state = self._paused
                            elif ev.get('t') == 'ctrl' and ev.get('a') == 'restart':
                                self._restart_requested = True
                                self._paused = False
                                paused_state = self._paused
                            else:
                                paused_state = self._paused
                                self._mouse_events.append(ev)
                        if ev.get('t') == 'ctrl' and ev.get('a') in ('toggle_pause', 'restart'):
                            await self._broadcast_json({'t': 'pause', 'paused': paused_state})
                    except json.JSONDecodeError:
                        pass
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    break
        finally:
            with self._lock:
                self._connected_clients.discard(ws)
        return ws

    async def _broadcast(self, jpeg_bytes):
        with self._lock:
            clients = list(self._connected_clients)
        for ws in clients:
            try:
                await ws.send_bytes(jpeg_bytes)
            except Exception:
                pass

    async def _broadcast_json(self, message):
        with self._lock:
            clients = list(self._connected_clients)
        for ws in clients:
            try:
                await ws.send_json(message)
            except Exception:
                pass

    # -- Mouse event handling --------------------------------------------------

    def _apply_mouse_event(self, ev):
        t = ev.get('t')
        if t == 'm':
            action_map = {
                'rotate': mujoco.mjtMouse.mjMOUSE_ROTATE_V,
                'move': mujoco.mjtMouse.mjMOUSE_MOVE_V,
                'zoom': mujoco.mjtMouse.mjMOUSE_ZOOM,
            }
            action = action_map.get(ev.get('a'))
            if action is None:
                return
            h = ev.get('h', 720)
            mujoco.mjv_moveCamera(
                self.model, action,
                ev.get('dx', 0) / h, ev.get('dy', 0) / h,
                self.scene, self.cam,
            )
        elif t == 's':
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                0, -0.05 * ev.get('dy', 0) / 120,
                self.scene, self.cam,
            )

    def _draw_overlay(self, frame, overlay_data):
        """Draw HUD overlay with robot state information on the frame.

        Wrapper that calls the shared overlay drawing function.

        Args:
            frame: BGR image (numpy array) to draw on
            overlay_data: dict with overlay data
        """
        _draw_overlay_on_frame(frame, overlay_data, column_width=180)

    def is_paused(self):
        with self._lock:
            return self._paused

    def consume_restart_request(self):
        """Return True once per restart request sent by a client."""
        with self._lock:
            if self._restart_requested:
                self._restart_requested = False
                return True
            return False


class PassiveViewer(RemoteViewer):
    """Local GLFW window viewer that inherits RemoteViewer's rendering pipeline.

    Uses the same offscreen rendering as RemoteViewer but displays frames in a
    local GLFW window instead of streaming over WebSocket. This ensures both
    viewers produce identical output.
    """

    def __init__(self, model, data, width=1200, height=900, title="MuJoCo Simulator",
                 overlay_callback=None):
        import glfw

        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")

        # Create window for display
        self.window = glfw.create_window(width, height, title, None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window")

        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # Mouse interaction state
        self._button_left = False
        self._button_middle = False
        self._button_right = False
        self._last_x = 0.0
        self._last_y = 0.0

        # Set GLFW callbacks
        glfw.set_mouse_button_callback(self.window, self._mouse_button_cb)
        glfw.set_cursor_pos_callback(self.window, self._mouse_move_cb)
        glfw.set_scroll_callback(self.window, self._scroll_cb)
        glfw.set_key_callback(self.window, self._key_cb)

        # Initialize parent RemoteViewer (with no WebSocket server since port=-1)
        # This sets up all the MuJoCo rendering infrastructure
        super().__init__(model, data, port=-1, width=width, height=height,
                        jpeg_quality=90, overlay_callback=overlay_callback)

        glfw.show_window(self.window)
        glfw.make_context_current(None)

    # -- GLFW callbacks -------------------------------------------------------

    def _mouse_button_cb(self, window, button, act, mods):
        import glfw
        self._button_left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        self._button_middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        self._button_right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        self._last_x, self._last_y = glfw.get_cursor_pos(window)

    def _mouse_move_cb(self, window, xpos, ypos):
        import glfw
        dx = xpos - self._last_x
        dy = ypos - self._last_y
        self._last_x = xpos
        self._last_y = ypos

        if not (self._button_left or self._button_middle or self._button_right):
            return

        _, height = glfw.get_window_size(window)

        if self._button_right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif self._button_left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM

        mujoco.mjv_moveCamera(self.model, action, dx / height, dy / height, self.scene, self.cam)
        self._dirty = True

    def _scroll_cb(self, window, xoffset, yoffset):
        mujoco.mjv_moveCamera(
            self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * yoffset, self.scene, self.cam
        )
        self._dirty = True

    def _key_cb(self, window, key, scancode, act, mods):
        import glfw
        if act == glfw.PRESS and key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)

    # -- Override parent's render to display in window instead of streaming ---

    def render(self):
        """Render a frame and display in GLFW window. Must be called from main thread."""
        import glfw

        if not self._running:
            return

        if glfw.window_should_close(self.window):
            self._running = False
            return

        glfw.poll_events()

        # Render frame using parent's core rendering logic
        if self._dirty or self._mouse_events:
            frame_bgr = self._render_to_buffer()

            if frame_bgr is not None:
                # Display in GLFW window
                glfw.make_context_current(self.window)

                # Convert BGR to RGB for OpenGL display
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                frame_flipped = np.flipud(frame_rgb)  # Flip for OpenGL

                # Display in window
                import OpenGL.GL as gl
                gl.glDrawPixels(self.width, self.height, gl.GL_RGB,
                               gl.GL_UNSIGNED_BYTE, frame_flipped.tobytes())

                glfw.swap_buffers(self.window)
                glfw.make_context_current(None)

    def _render_to_buffer(self):
        """Render frame to buffer with overlay. Returns BGR frame."""
        # Drain pending mouse events and apply to camera
        with self._lock:
            events = self._mouse_events
            self._mouse_events = []
        for ev in events:
            self._apply_mouse_event(ev)

        if not self._dirty and not events:
            return None

        self._gl_ctx.make_current()

        mujoco.mjv_updateScene(
            self.model, self.data, self.opt, self.pert, self.cam,
            mujoco.mjtCatBit.mjCAT_ALL, self.scene,
        )
        mujoco.mjr_render(self._viewport, self.scene, self.con)
        mujoco.mjr_readPixels(self._rgb, None, self._viewport, self.con)

        # OpenGL renders bottom-up; flip, then BGR for cv2
        frame = np.flipud(self._rgb)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Draw overlay if callback is provided
        if self._overlay_callback is not None:
            try:
                overlay_data = self._overlay_callback()
                self._draw_overlay(frame_bgr, overlay_data)
            except Exception as e:
                pass  # Silently ignore overlay errors

        self._dirty = False
        return frame_bgr

    def is_running(self):
        import glfw
        if not self._running:
            return False
        if glfw.window_should_close(self.window):
            self._running = False
            return False
        return True

    def close(self):
        import glfw
        self._running = False
        if hasattr(self, 'window') and self.window:
            glfw.destroy_window(self.window)
            self.window = None
        # Don't call parent's close() since it would try to stop the WebSocket server
        # which doesn't exist for PassiveViewer


class MuJoCoVisualizer:

    def __init__(self, render_fps: int = 50, render_size: (int, int) = (640, 640)):
        """

        @param render_fps:
        @param render_size:
        """
        self.mjc_model = None
        self.mjc_data = None
        self.renderer = None
        self.scene = None
        self.data = {}
        self.render_fps = render_fps
        self.render_size = render_size
        self.camera = "fixed"

    def set_camera(self, camera_name: str):
        self.camera = camera_name

    def set_xml_path(self, xml_path: Path):
        self.mjc_model = self._load_model_from_xml(xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)
        self.renderer = mujoco.Renderer(self.mjc_model, self.render_size[0], self.render_size[1])

        mujoco.mj_resetData(self.mjc_model, self.mjc_data)

    def _load_model_from_xml(self, xml_path: Path) -> mujoco.MjModel:
        model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
        return model

    def load_data(self, data_path: Path):
        with data_path.open("r") as fp:
            self.data = json.load(fp)

    def visualize(self, save_video_path: Path, dt: float):
        frames = []
        num_steps_per_frame = int(1 / self.render_fps / dt)
        for i, data_step in tqdm.tqdm(enumerate(self.data)):
            # if True:
            if i % num_steps_per_frame == 0:
                frame = self.take_snap_shot(data_step['time'],
                                            data_step['pose'])

                frames.append(frame)
                # cv2.imwrite(Path(save_video_path, f"{i}.png").as_posix(), frame)

        self.save_video(save_video_path, frames)

    def visualize_from_ext_data(self,
                                xml_path: Path,
                                data_path: Path,
                                dt: float,
                                video_path: Path):
        self.mjc_model = self._load_model_from_xml(xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)
        self.renderer = mujoco.Renderer(self.mjc_model, self.render_size[0], self.render_size[1])
        self.load_data(data_path)

        self.visualize(video_path, dt)

    def render_frame(self):
        self.renderer.update_scene(self.mjc_data, self.camera)
        frame = self.renderer.render().copy()
        return frame

    def save_video(self, save_path: Path, frames: list):
        frame_size = (self.renderer.width, self.renderer.height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(save_path.as_posix(), fourcc, self.render_fps, frame_size)

        for i, frame in enumerate(frames):
            im = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            video_writer.write(im)

        video_writer.release()

    def take_snap_shot(self, t: float = None, pos: np.array = None, camera_view: str = None):
        if t:
            self.mjc_data.time = t
        if pos is not None:
            self.mjc_data.qpos = pos

        mujoco.mj_forward(self.mjc_model, self.mjc_data)
        self.renderer.update_scene(self.mjc_data, camera_view if camera_view else self.camera)
        frame = self.renderer.render().copy()

        return frame

    def add_line_segment(self, scene, point1, point2, radius, rgba):
        """Adds one capsule to an mjvScene."""
        if scene.ngeom >= scene.maxgeom:
            print("Max scene geoms reached")
            return
        scene.ngeom += 1  # increment ngeom
        # initialise a new capsule, add it to the scene using mjv_connector
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom - 1],
                            mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                            np.zeros(3), np.zeros(9), rgba.astype(np.float32))
        mujoco.mjv_makeConnector(scene.geoms[scene.ngeom - 1],
                                 mujoco.mjtGeom.mjGEOM_CAPSULE,
                                 radius,
                                 point1[0], point1[1], 0.5,
                                 point2[0], point2[1], 0.5
                                 )

    def add_capsule(self, scene, pt, radius=0.3, rgba=np.ones(4)):
        if scene.ngeom >= scene.maxgeom:
            print("Max scene geoms reached")
            return
        scene.ngeom += 1  # increment ngeom
        # initialise a new capsule, add it to the scene using mjv_connector
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom - 1],
                            mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius, radius, radius]),
                            pt, np.zeros(9), rgba.astype(np.float32)
                            )

    def add_path_to_scene(self, positions, radius=0.05, rgba=None):
        """Draw position trace, speed modifies width and colors."""
        if rgba is None:
            rgba = np.array([1.0, 0., 0., 1.0])

        for i in range(len(positions) - 1):
            pt1 = positions[i]
            pt2 = positions[i + 1]
            self.add_line_segment(self.renderer.scene, pt1, pt2, radius, rgba)


def stream_trajectory_from_json(json_data_path: str | Path,
                                xml_path: str | Path,
                                gt_data_path: str | Path | None = None,
                                port: int = 8765,
                                width: int = 1920,
                                height: int = 1280,
                                playback_speed: float = 1.0):
    """Stream a trajectory from JSON data through a RemoteViewer in real time.

    Args:
        json_data_path: Path to a JSON file containing a list of dicts,
            each with 'time' (float) and 'pos' (list of floats for qpos).
        xml_path: Path to the MuJoCo XML model file.
        port: WebSocket server port for the RemoteViewer.
        width: Render width in pixels.
        height: Render height in pixels.
        playback_speed: Multiplier for playback speed (1.0 = real time).
    """
    json_data_path = Path(json_data_path)
    xml_path = Path(xml_path)

    gt_data = None
    if gt_data_path:
        with Path(gt_data_path).open() as f:
            gt_data = json.load(f)

    with json_data_path.open("r") as f:
        trajectory = json.load(f)

    model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
    data = mujoco.MjData(model)

    total_time = trajectory[-1]['time'] - trajectory[0]['time']
    overlay_data = {'time_text': f"Time: 0.00 / {total_time:.2f} s"}

    viewer = RemoteViewer(
        model,
        data,
        port=port,
        width=width,
        height=height,
        overlay_callback=lambda: overlay_data,
    )
    with viewer._lock:
        viewer._paused = False
    print(f"RemoteViewer started — open http://localhost:{port} in your browser")

    try:
        while viewer.is_running():
            wall_start = time.monotonic()
            sim_start = trajectory[0]['time']
            restart_requested = False

            for i, step in enumerate(trajectory):
                if not viewer.is_running():
                    break
                if viewer.consume_restart_request():
                    restart_requested = True
                    break

                while viewer.is_running() and viewer.is_paused():
                    paused_start = time.monotonic()
                    while viewer.is_running() and viewer.is_paused():
                        if viewer.consume_restart_request():
                            restart_requested = True
                            break
                        time.sleep(0.05)
                    if restart_requested or not viewer.is_running():
                        break
                    wall_start += time.monotonic() - paused_start

                if restart_requested or not viewer.is_running():
                    break

                sim_elapsed = (step['time'] - sim_start) / playback_speed
                wall_elapsed = time.monotonic() - wall_start
                sleep = sim_elapsed - wall_elapsed
                if sleep > 0:
                    time.sleep(sleep)

                pose = np.array(step['pose'])
                if gt_data:
                    d = gt_data[i]
                    gt_pos, gt_q = np.array(d['pos']), np.array(d['quat'])
                    gt_pose = np.hstack([gt_pos.reshape(-1, 3), gt_q.reshape(-1, 4)]).flatten()
                    pose = np.concatenate([pose, gt_pose])

                data.time = step['time']
                data.qpos = pose
                mujoco.mj_forward(model, data)
                elapsed_time = step['time'] - sim_start
                overlay_data['time_text'] = f"Time: {elapsed_time:.2f} / {total_time:.2f} s"
                viewer.sync()

            if not viewer.is_running():
                break

            if restart_requested:
                overlay_data['time_text'] = f"Time: 0.00 / {total_time:.2f} s"
                viewer.sync()
                continue

            # Keep viewer alive after playback so user can still interact and optionally restart.
            print("Playback finished — viewer still running (Ctrl+C to exit, Restart to replay)")
            while viewer.is_running():
                if viewer.consume_restart_request():
                    overlay_data['time_text'] = f"Time: 0.00 / {total_time:.2f} s"
                    viewer.sync()
                    restart_requested = True
                    break
                time.sleep(0.1)

            if not restart_requested:
                break
    except KeyboardInterrupt:
        print("\nShutting down viewer")
    finally:
        viewer.close()


def save_trajectory_video_from_json(json_data_path: str | Path,
                                    xml_path: str | Path,
                                    output_video_path: str | Path,
                                    gt_data_path: str | Path | None = None,
                                    width: int = 960,
                                    height: int = 720,
                                    playback_speed_multiplier: int = 1):
    """Render a trajectory from JSON data and save it as an MP4 video.

    Args:
        json_data_path: Path to a JSON file containing a list of dicts,
            each with 'time' (float) and 'pose' (list of floats for qpos).
        xml_path: Path to the MuJoCo XML model file.
        output_video_path: Destination path for the .mp4 video.
        gt_data_path: Optional path to GT data JSON with pos/quat fields.
        width: Render width in pixels.
        height: Render height in pixels.
        playback_speed: Speed multiplier for output timing (1.0 = original speed).
        output_fps: Optional output FPS override. If omitted, derived from trajectory.
    """
    json_data_path = Path(json_data_path)
    xml_path = Path(xml_path)
    output_video_path = Path(output_video_path)
    output_video_path.parent.mkdir(parents=True, exist_ok=True)

    with json_data_path.open("r") as f:
        trajectory = json.load(f)

    if not trajectory:
        raise ValueError(f"Trajectory file is empty: {json_data_path}")

    gt_data = None
    if gt_data_path:
        with Path(gt_data_path).open() as f:
            gt_data = json.load(f)

        for i, pose_dict in enumerate(trajectory):
            pose = np.array(pose_dict['pose'])
            if gt_data:
                gt_pos, gt_q = np.array(gt_data[i]['pos']), np.array(gt_data[i]['quat'])
                gt_pose = np.hstack([gt_pos.reshape(-1, 3), gt_q.reshape(-1, 4)]).flatten()
                pose = np.concatenate([pose, gt_pose])
            trajectory[i]['pose'] = pose

    visualizer = MuJoCoVisualizer()
    visualizer.set_xml_path(xml_path)
    visualizer.set_camera("camera")
    visualizer.data = trajectory[::playback_speed_multiplier]
    visualizer.visualize(output_video_path, 0.01)

    print(f"Saved video to: {output_video_path}")


def create_trajectory_image(xml_path: str | Path,
                            poses_json_path: str | Path,
                            output_path: str | Path):
    """Interactively build a composite trajectory image from selected pose crops.

    Args:
        xml_path: Path to MuJoCo XML model.
        poses_json_path: Path to JSON trajectory data. Each item can be:
            - {'pose': [...], 'time': ...}
            - {'pos': [...], 'time': ...}
            - [...] (raw qpos list)
        output_path: Destination image path for the final composite.

    Returns:
        List[Dict]: Captured selections with keys {'pose_index', 'bbox'} where
            bbox is [x_min, y_min, x_max, y_max] in pixel coordinates.
    """
    xml_path = Path(xml_path)
    poses_json_path = Path(poses_json_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with poses_json_path.open("r") as fp:
        pose_data = json.load(fp)

    if not pose_data:
        raise ValueError(f"No poses found in {poses_json_path}")

    def _extract_pose(entry):
        if isinstance(entry, dict):
            if "pose" in entry:
                return np.array(entry["pose"])
            if "pos" in entry:
                return np.array(entry["pos"])
            raise KeyError("Pose entry dict must contain 'pose' or 'pos'")
        return np.array(entry)

    visualizer = MuJoCoVisualizer()
    visualizer.set_xml_path(xml_path)
    visualizer.set_camera("camera")

    initial_image = visualizer.take_snap_shot(pos=_extract_pose(pose_data[0]), camera_view="camera")
    img_h, img_w = initial_image.shape[:2]

    selected_items = []

    fig, ax = plt.subplots(figsize=(8, 8))
    plt.subplots_adjust(left=0.18, bottom=0.36, right=0.95)
    ax.set_title("Trajectory Composite Selector")
    ax.axis("off")

    displayed_rgb = initial_image.copy()
    image_artist = ax.imshow(displayed_rgb)

    bbox_patch = plt.Rectangle((0, 0), img_w, img_h, fill=False, edgecolor="red", linewidth=2)
    ax.add_patch(bbox_patch)

    slider_pose_ax = plt.axes([0.18, 0.27, 0.70, 0.03])
    slider_xmin_ax = plt.axes([0.18, 0.22, 0.70, 0.03])
    slider_ymin_ax = plt.axes([0.18, 0.17, 0.70, 0.03])
    slider_xmax_ax = plt.axes([0.18, 0.12, 0.70, 0.03])
    slider_ymax_ax = plt.axes([0.18, 0.07, 0.70, 0.03])
    button_save_ax = plt.axes([0.18, 0.01, 0.20, 0.045])

    pose_slider = Slider(
        slider_pose_ax,
        "pose idx",
        0,
        len(pose_data) - 1,
        valinit=0,
        valstep=1,
    )
    xmin_slider = Slider(slider_xmin_ax, "x_min", 0, img_w - 1, valinit=0, valstep=1)
    ymin_slider = Slider(slider_ymin_ax, "y_min", 0, img_h - 1, valinit=0, valstep=1)
    xmax_slider = Slider(slider_xmax_ax, "x_max", 0, img_w - 1, valinit=img_w - 1, valstep=1)
    ymax_slider = Slider(slider_ymax_ax, "y_max", 0, img_h - 1, valinit=img_h - 1, valstep=1)
    save_button = Button(button_save_ax, "Save Selection")

    current_state = {"pose_index": 0, "image": displayed_rgb}

    def _get_bbox():
        x0 = int(min(xmin_slider.val, xmax_slider.val))
        x1 = int(max(xmin_slider.val, xmax_slider.val))
        y0 = int(min(ymin_slider.val, ymax_slider.val))
        y1 = int(max(ymin_slider.val, ymax_slider.val))
        return x0, y0, x1, y1

    def _refresh(_):
        pose_index = int(pose_slider.val)
        pose = _extract_pose(pose_data[pose_index])
        frame = visualizer.take_snap_shot(pos=pose, camera_view="camera")
        current_state["pose_index"] = pose_index
        current_state["image"] = frame

        image_artist.set_data(frame)

        x0, y0, x1, y1 = _get_bbox()
        bbox_patch.set_xy((x0, y0))
        bbox_patch.set_width(max(1, x1 - x0))
        bbox_patch.set_height(max(1, y1 - y0))
        fig.canvas.draw_idle()

    def _save_selection(_):
        x0, y0, x1, y1 = _get_bbox()
        selected_items.append({
            "pose_index": int(current_state["pose_index"]),
            "bbox": [x0, y0, x1, y1],
            "image": current_state["image"].copy(),
        })
        print(f"Saved selection #{len(selected_items)}: pose={current_state['pose_index']} bbox={[x0, y0, x1, y1]}")

    pose_slider.on_changed(_refresh)
    xmin_slider.on_changed(_refresh)
    ymin_slider.on_changed(_refresh)
    xmax_slider.on_changed(_refresh)
    ymax_slider.on_changed(_refresh)
    save_button.on_clicked(_save_selection)

    _refresh(None)
    plt.show()
    plt.close(fig)

    if not selected_items:
        # Default to one full-frame selection from pose 0.
        selected_items.append({
            "pose_index": 0,
            "bbox": [0, 0, img_w - 1, img_h - 1],
            "image": initial_image.copy(),
        })

    alpha = 0.5  # Patch opacity when blending onto initial image.
    composite = initial_image.copy()
    for item in selected_items:
        x0, y0, x1, y1 = item["bbox"]
        base_roi = composite[y0:y1 + 1, x0:x1 + 1].astype(np.float32)
        patch_roi = item["image"][y0:y1 + 1, x0:x1 + 1].astype(np.float32)
        blended_roi = ((1.0 - alpha) * base_roi) + (alpha * patch_roi)
        composite[y0:y1 + 1, x0:x1 + 1] = np.clip(blended_roi, 0, 255).astype(np.uint8)

    output_bgr = cv2.cvtColor(composite, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(output_path.as_posix(), output_bgr):
        raise RuntimeError(f"Failed to save output image to {output_path}")

    print(f"Saved trajectory composite image to: {output_path}")
    return [{"pose_index": i["pose_index"], "bbox": i["bbox"]} for i in selected_items]


if __name__ == '__main__':
    for i in range(2, 5):
        xml_path = Path("xml/3prism_real_upscaled_vis.xml")
        base_path = Path("../../../tensegrity/data_sets/tensegrity_real_datasets/models/rss_demo_new_v1/"
                         )
        frames = json.load((base_path / f"patrick{i}_init_frames.json").open('r'))[::2]

        visualizer = MuJoCoVisualizer()
        visualizer.set_xml_path(Path(xml_path))
        visualizer.data = frames
        visualizer.set_camera("front")
        # visualizer.visualize(Path(model_path, f"{base_path.name}_vid_camera.mp4"), dt)
        # visualizer.set_camera("front")
        visualizer.visualize(Path(base_path, f"vid{i}.mp4"), 0.01)

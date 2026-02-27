import json
import random
import threading
import asyncio
import time
from pathlib import Path
from typing import List, Optional

import cv2
import mujoco
import numpy as np

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
</style></head><body>
<canvas id="v"></canvas><div id="hud">Connecting...</div>
<script>
const C=document.getElementById('v'),X=C.getContext('2d'),H=document.getElementById('hud');
let ws,fc=0,ft=performance.now();
function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.binaryType='arraybuffer';
  ws.onopen=()=>{H.textContent='Connected'};
  ws.onclose=()=>{H.textContent='Reconnecting...';setTimeout(connect,1000)};
  ws.onerror=()=>{};
  ws.onmessage=e=>{
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
        return aiohttp.web.Response(text=_VIEWER_HTML, content_type='text/html')

    async def _handle_ws(self, request):
        ws = aiohttp.web.WebSocketResponse()
        await ws.prepare(request)

        with self._lock:
            self._connected_clients.add(ws)

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        ev = json.loads(msg.data)
                        with self._lock:
                            self._mouse_events.append(ev)
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


class AbstractMuJoCoSimulator:
    """
    MuJoCo spring rod simulator
    """

    def __init__(self,
                 xml_path: Path | str,
                 visualize: bool = False,
                 render_size: (int, int) = (1280, 1280),
                 render_fps: int = 100,
                 use_remote_viewer: bool = False,
                 remote_port: int = 8765,
                 remote_max_fps: int = None,
                 overlay_callback=None):
        self.xml_path = Path(xml_path)
        self.visualize = visualize
        self.mjc_model = self._load_model_from_xml(self.xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)
        self.renderer = mujoco.Renderer(self.mjc_model, render_size[0], render_size[1]) if visualize else None
        self.render_fps = render_fps
        self.states = []
        self.time = 0
        self.dt = self.mjc_model.opt.timestep

        # Remote viewer
        self.remote_viewer = None
        if use_remote_viewer:
            self.remote_viewer = RemoteViewer(
                self.mjc_model, self.mjc_data, port=remote_port,
                overlay_callback=overlay_callback, max_fps=remote_max_fps
            )
            print(f"[RemoteViewer] Open http://localhost:{remote_port} in your browser")

    def reset(self):
        self.mjc_model = self._load_model_from_xml(self.xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)

    def _load_model_from_xml(self, xml_path: Path) -> mujoco.MjModel:
        model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
        return model

    def sim_step(self):
        # self.mjc_data.ctrl = [-500, -500]
        # self.mjc_model.tendon_lengthspring = np.maximum(self.mjc_model.tendon_lengthspring - 0.01, 0.0)
        mujoco.mj_step(self.mjc_model, self.mjc_data)
        self.sync_remote_viewer()
        # k=1

    def forward(self):
        mujoco.mj_forward(self.mjc_model, self.mjc_data)

    def render_frame(self, view='camera'):
        self.renderer.update_scene(self.mjc_data, view)
        frame = self.renderer.render().copy()
        return frame

    def render_frames_from_poses(self, poses, view='camera'):
        curr_pose = self.mjc_model.qpos().copy()
        frames = []
        for pose in poses:
            self.mjc_data.qpos = pose
            self.forward()
            frame = self.render_frame(view)
            frames.append(frame)

        self.mjc_data.qpos = curr_pose
        self.forward()
        return frames

    def sync_remote_viewer(self):
        """Sync and render the remote viewer if active."""
        if self.remote_viewer is not None and self.remote_viewer.is_running():
            self.remote_viewer.sync()
            self.remote_viewer.render()

    def run(self,
            end_time: float = None,
            num_steps: int = None,
            save_path: Path = None,
            pos_sensor_names: Optional[List] = None,
            quat_sensor_names: Optional[List] = None,
            linvel_sensor_names: Optional[List] = None,
            angvel_sensor_names: Optional[List] = None):

        if end_time is None and num_steps is None:
            raise Exception("Need to specify one of time params (end_time or num_steps)")
        elif end_time:
            num_steps = np.ceil(end_time / self.dt).astype(int)

        frames = []
        num_steps_per_frame = int(1 / self.render_fps / self.dt)

        poses, end_pts = [], []
        for i in range(num_steps + 1):
            mujoco.mj_forward(self.mjc_model, self.mjc_data)
            # self.mjc_data.ctrl = 0.1
            # if self.visualize and ((i + 1) % num_steps_per_frame == 0 or i == num_steps - 1):
            #     frame = self.render_frame()
            #     frames.append(frame.copy())

            if i % 100:
                print(f"Timestep: {self.mjc_data.time}")

            pos = self.mjc_data.qpos.tolist()
            # end_pt = np.hstack([self.mjc_data.sensor("pos_s0").data.copy(), self.mjc_data.sensor("pos_s1").data.copy()])
            poses.append(pos.copy())
            # end_pts.append(end_pt)
            vel = self.mjc_data.qvel.tolist()
            self.states.append({"time": round(self.mjc_data.time, 5),
                                "pos": pos,
                                "vel": vel,
                                "r01_end_pt1": self.mjc_data.sensor("pos_s0").data.tolist(),
                                "r01_end_pt2": self.mjc_data.sensor("pos_s1").data.tolist(),
            #                     "r23_end_pt1": self.mjc_data.sensor("pos_s2").data.tolist(),
            #                     "r23_end_pt2": self.mjc_data.sensor("pos_s3").data.tolist(),
            #                     "r45_end_pt1": self.mjc_data.sensor("pos_s4").data.tolist(),
            #                     "r45_end_pt2": self.mjc_data.sensor("pos_s5").data.tolist(),
            #                     "s6": self.mjc_data.sensor("pos_s6").data.tolist(),
            #                     "s7": self.mjc_data.sensor("pos_s7").data.tolist(),
            #                     "s8": self.mjc_data.sensor("pos_s8").data.tolist(),
            #                     "s9": self.mjc_data.sensor("pos_s9").data.tolist(),
            #                     "s10": self.mjc_data.sensor("pos_s10").data.tolist(),
            #                     "s11": self.mjc_data.sensor("pos_s11").data.tolist(),
            #                     "s12": self.mjc_data.sensor("pos_s12").data.tolist(),
            #                     "s13": self.mjc_data.sensor("pos_s13").data.tolist(),
            #                     "s14": self.mjc_data.sensor("pos_s14").data.tolist(),
            #                     "s15": self.mjc_data.sensor("pos_s15").data.tolist(),
            #                     "s16": self.mjc_data.sensor("pos_s16").data.tolist(),
            #                     "s17": self.mjc_data.sensor("pos_s17").data.tolist(),
            #                     "s18": self.mjc_data.sensor("pos_s18").data.tolist(),
            #                     "s19": self.mjc_data.sensor("pos_s19").data.tolist(),
            #                     "s20": self.mjc_data.sensor("pos_s20").data.tolist(),
            #                     "s21": self.mjc_data.sensor("pos_s21").data.tolist(),
            #                     "s22": self.mjc_data.sensor("pos_s22").data.tolist(),
            #                     "s23": self.mjc_data.sensor("pos_s23").data.tolist(),
            #                     # "rod_3_end_pt1": self.mjc_data.sensor("pos_s6").data.tolist(),
            #                     # "rod_3_end_pt2": self.mjc_data.sensor("pos_s7").data.tolist(),
            #                     # "rod_4_end_pt1": self.mjc_data.sensor("pos_s8").data.tolist(),
            #                     # "rod_4_end_pt2": self.mjc_data.sensor("pos_s9").data.tolist(),
            #                     # "rod_5_end_pt1": self.mjc_data.sensor("pos_s10").data.tolist(),
            #                     # "rod_5_end_pt2": self.mjc_data.sensor("pos_s11").data.tolist()
                                })
            self.sim_step()

        if save_path:
            save_path = Path(save_path)
            save_path.mkdir(exist_ok=True)

            with Path(save_path, "data.json").open("w") as fp:
                json.dump(self.states, fp)

            if self.visualize:
                self.save_video(save_path / "video.mp4", frames)

        self.states.clear()
        return end_pts, poses

    def save_video(self, save_path: Path, frames: list):
        frame_size = (self.renderer.width, self.renderer.height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(save_path.as_posix(), fourcc, self.render_fps, frame_size)

        for i, frame in enumerate(frames):
            im = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # cv2.imwrite(f"tmp/frame{i}.png", frame)
            video_writer.write(im)

        video_writer.release()


if __name__ == '__main__':
    import shutil

    base_path = Path('/Users/nelsonchen/Documents/Rutgers-CS-PhD/Research/tensegrity/data_sets/')
    exp_dir = Path('/Users/nelsonchen/Desktop/tmp_data/')
    xml_path = Path("xml_models/single_rod_spring.xml")
    exp_dir.mkdir(exist_ok=True)

    sim = AbstractMuJoCoSimulator(xml_path, visualize=True)
    sim.run(num_steps=400, save_path=exp_dir)

import glfw
from OpenGL.GL import *
import numpy as np

trans = np.identity(3)

def render(T):
    glClear(GL_COLOR_BUFFER_BIT)
    glLoadIdentity()
    # draw coordinate
    glBegin(GL_LINES)
    glColor3ub(255, 0, 0)
    glVertex2fv(np.array([0., 0.]))
    glVertex2fv(np.array([1., 0.]))
    glColor3ub(0, 255, 0)
    glVertex2fv(np.array([0., 0.]))
    glVertex2fv(np.array([0., 1.]))
    glEnd()
    # draw triangle
    glBegin(GL_TRIANGLES)
    glColor3ub(255, 255, 255)
    glVertex2fv((T @ np.array([.0, .5, 1.]))[:-1])
    glVertex2fv((T @ np.array([.0, .0, 1.]))[:-1])
    glVertex2fv((T @ np.array([.5, .0, 1.]))[:-1])
    glEnd()

def key_callback(window, key, scancode, action, mods):
    global trans
    deg = np.pi / 18

    if action in (glfw.REPEAT, glfw.RELEASE):
        return

    if key==glfw.KEY_Q:
        M = np.identity(3)
        M[0, 2] = -0.1
        trans = M @ trans
    elif key==glfw.KEY_E:
        M = np.identity(3)
        M[0, 2] = 0.1
        trans = M @ trans
    elif key==glfw.KEY_A:
        R = np.identity(3)
        R[:2, :2] = np.array([[np.cos(deg), -np.sin(deg)], [np.sin(deg), np.cos(deg)]])
        trans = trans @ R
    elif key==glfw.KEY_D:
        R = np.identity(3)
        deg = -deg
        R[:2, :2] = np.array([[np.cos(deg), -np.sin(deg)], [np.sin(deg), np.cos(deg)]])
        trans = trans @ R
    elif key==glfw.KEY_1:
        trans = np.identity(3)

def main():
    global trans
    if not glfw.init():
        return
    window = glfw.create_window(480, 480, "2023003227-3-1", None, None)
    if not window:
        glfw.terminate()
        return

    glfw.set_key_callback(window, key_callback)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        
        render(trans)

        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()



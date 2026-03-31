import glfw
from OpenGL.GL import *
import numpy as np

shape_type = GL_LINE_LOOP

def render():
    glClear(GL_COLOR_BUFFER_BIT)
    glLoadIdentity()
    glBegin(shape_type)
    for i in np.arange(0, 2*np.pi, np.pi/6):
        glVertex2f(np.cos(i), np.sin(i))
    glEnd()

def key_callback(window, key, scancode, action, mods):
    global shape_type

    if key==glfw.KEY_1:
        shape_type = GL_POINTS
    elif key==glfw.KEY_2:
        shape_type = GL_LINES
    elif key==glfw.KEY_3:
        shape_type = GL_LINE_STRIP
    elif key==glfw.KEY_4:
        shape_type = GL_LINE_LOOP
    elif key==glfw.KEY_5:
        shape_type = GL_TRIANGLES
    elif key==glfw.KEY_6:
        shape_type = GL_TRIANGLE_STRIP
    elif key==glfw.KEY_7:
        shape_type = GL_TRIANGLE_FAN
    elif key==glfw.KEY_8:
        shape_type = GL_QUADS
    elif key==glfw.KEY_9:
        shape_type = GL_QUAD_STRIP
    elif key==glfw.KEY_0:
        shape_type = GL_POLYGON


def main():
    if not glfw.init():
        return
    
    window = glfw.create_window(480, 480, "2023003227-2-1", None, None)
    if not window:
        glfw.terminate()
        return
    
    glfw.set_key_callback(window, key_callback)
    glfw.make_context_current(window)

    while not glfw.window_should_close(window):
        glfw.poll_events()

        render()

        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()

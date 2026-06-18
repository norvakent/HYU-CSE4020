import glfw
import sys
import pdb
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.arrays import ArrayDatatype
import time
import numpy as np
import ctypes
from PIL.Image import open
import OBJ
from Ray import *


# global variables
wld2cam=[]
cam2wld=[]
cow2wld=None
cursorOnCowBoundingBox=False
pickInfo=None
floorTexID=0
cameras= [
	[28,18,28, 0,2,0, 0,1,0],   
	[28,18,-28, 0,2,0, 0,1,0], 
	[-28,18,28, 0,2,0, 0,1,0], 
	[-12,12,0, 0,2,0, 0,1,0],  
	[0,100,0,  0,0,0, 1,0,0]
]
camModel=None
cowModel=None
H_DRAG=1
V_DRAG=2
# dragging state
isDrag=0

# Cow roller-coaster state
CONTROL_POINT_COUNT = 6
LAP_COUNT = 3
SEGMENT_DURATION = 1.0       # seconds per B-spline segment
controlPoints = []           # each element is a 4x4 cow-to-world matrix
placementStarted = False     # True after the initial cow pick
recordPointOnRelease = False
animating = False
animStartTime = 0.0
verticalDragPlane = None
verticalDragStartConfig = None
verticalDragStartPick = None
lastCowYaw = -np.pi / 2.0     # initial cow direction: local +X -> world +Z

class PickInfo:
    def __init__(self, cursorRayT, cowPickPosition, cowPickConfiguration, cowPickPositionLocal):
        self.cursorRayT=cursorRayT
        self.cowPickPosition=cowPickPosition.copy()
        self.cowPickConfiguration=cowPickConfiguration.copy()
        self.cowPickPositionLocal=cowPickPositionLocal.copy()

def vector3(x,y,z):
    return np.array((x,y,z))
def position3(v):
    # divide by w
    w=v[3]
    return vector3(v[0]/w, v[1]/w, v[2]/w)

def vector4(x,y,z):
    return np.array((x,y,z,1))

def rotate(m,v):
    return m[0:3, 0:3]@v
def transform(m, v):
    return position3(m@np.append(v,1))

def getTranslation(m):
    return m[0:3,3]
def setTranslation(m,v):
    m[0:3,3]=v


def cubicBSplinePoint(Pm1, P0, P1, P2, u):
    """Uniform cubic B-spline point (approximating spline)."""
    u2 = u * u
    u3 = u2 * u
    return (
        (-u3 + 3.0*u2 - 3.0*u + 1.0) * Pm1
        + (3.0*u3 - 6.0*u2 + 4.0) * P0
        + (-3.0*u3 + 3.0*u2 + 3.0*u + 1.0) * P1
        + u3 * P2
    ) / 6.0


def cubicBSplineTangent(Pm1, P0, P1, P2, u):
    """Derivative of cubicBSplinePoint with respect to u."""
    u2 = u * u
    return (
        (-3.0*u2 + 6.0*u - 3.0) * Pm1
        + (9.0*u2 - 12.0*u) * P0
        + (-9.0*u2 + 6.0*u + 3.0) * P1
        + (3.0*u2) * P2
    ) / 6.0


def splinePositionAndTangent(points, segmentIndex, u):
    """Evaluate one segment of a cyclic uniform cubic B-spline."""
    n = len(points)
    positions = [getTranslation(M) for M in points]
    Pm1 = positions[(segmentIndex - 1) % n]
    P0  = positions[segmentIndex % n]
    P1  = positions[(segmentIndex + 1) % n]
    P2  = positions[(segmentIndex + 2) % n]
    return (
        cubicBSplinePoint(Pm1, P0, P1, P2, u),
        cubicBSplineTangent(Pm1, P0, P1, P2, u)
    )


def makeCowTransform(position, tangent):
    """Create translation + yaw/pitch transform; cow's local +X is forward."""
    global lastCowYaw

    direction = normalize(tangent)
    horizontalLength = np.hypot(direction[0], direction[2])

    # yaw: turn local +X toward the tangent's XZ direction
    if horizontalLength > 1e-8:
        lastCowYaw = np.arctan2(-direction[2], direction[0])

    # pitch: positive while climbing, negative while descending
    pitch = np.arctan2(direction[1], horizontalLength)

    cy, sy = np.cos(lastCowYaw), np.sin(lastCowYaw)
    cp, sp = np.cos(pitch), np.sin(pitch)

    Ry = np.array([
        [ cy, 0.0, sy],
        [0.0, 1.0, 0.0],
        [-sy, 0.0, cy]
    ])
    Rz = np.array([
        [cp, -sp, 0.0],
        [sp,  cp, 0.0],
        [0.0, 0.0, 1.0]
    ])

    M = np.eye(4)
    M[0:3, 0:3] = Ry @ Rz
    setTranslation(M, position)
    return M


def drawSplineTrack(points, samplesPerSegment=30):
    """Draw the closed B-spline track after all six points are available."""
    if len(points) < CONTROL_POINT_COUNT:
        return

    glDisable(GL_LIGHTING)
    glLineWidth(3.0)
    glColor3d(0.15, 0.15, 0.15)
    glBegin(GL_LINE_LOOP)
    for segmentIndex in range(len(points)):
        for sample in range(samplesPerSegment):
            u = sample / float(samplesPerSegment)
            position, _ = splinePositionAndTangent(points, segmentIndex, u)
            glVertex3d(position[0], position[1], position[2])
    glEnd()
    glLineWidth(1.0)


def setHorizontalPickAnchor(window):
    """Reset the mouse anchor so horizontal following continues smoothly."""
    global pickInfo

    x, y = glfw.get_cursor_pos(window)
    ray = screenCoordToRay(window, x, y)
    cowPosition = getTranslation(cow2wld)
    plane = Plane(np.array((0.0, 1.0, 0.0)), cowPosition)
    hit, t = ray.intersectsPlane(plane)
    if not hit:
        pickInfo = None
        return

    anchorWorld = ray.getPoint(t)
    anchorLocal = transform(np.linalg.inv(cow2wld), anchorWorld)
    pickInfo = PickInfo(t, anchorWorld, cow2wld, anchorLocal)


def startVerticalDrag(window, x, y):
    """Create a screen-facing drag plane and save the initial cow transform."""
    global verticalDragPlane, verticalDragStartConfig, verticalDragStartPick

    ray = screenCoordToRay(window, x, y)
    verticalDragStartConfig = cow2wld.copy()

    if pickInfo is not None:
        verticalDragStartPick = transform(cow2wld, pickInfo.cowPickPositionLocal)
    else:
        verticalDragStartPick = getTranslation(cow2wld).copy()

    # Plane normal is the initial screen ray: the plane is perpendicular
    # to the viewing direction and therefore behaves like a screen plane.
    verticalDragPlane = Plane(ray.direction, verticalDragStartPick)

def makePlane( a,  b,  n):
    v=a.copy()
    for i in range(3):
        if n[i]==1.0:
            v[i]=b[i];
        elif n[i]==-1.0:
            v[i]=a[i];
        else:
            assert(n[i]==0.0);
            
    return Plane(rotate(cow2wld,n),transform(cow2wld,v));

def onKeyPress(window, key, scancode, action, mods):
    global cameraIndex
    if action == glfw.RELEASE:
        return

    if key == glfw.KEY_C or key == glfw.KEY_SPACE:
        print("Toggle camera %s" % cameraIndex)
        cameraIndex = (cameraIndex + 1) % len(wld2cam)

        # Prevent a position jump when the viewpoint changes while the cow
        # is following the cursor.
        if placementStarted and not animating and isDrag == H_DRAG:
            setHorizontalPickAnchor(window)

def drawOtherCamera():
    global cameraIndex,wld2cam, camModel
    for i in range(len(wld2cam)):
        if (i != cameraIndex):
            glPushMatrix();												# Push the current matrix on GL to stack. The matrix is wld2cam[cameraIndex].matrix().
            glMultMatrixd(cam2wld[i].T)
            drawFrame(5);											# Draw x, y, and z axis.
            frontColor = [0.2, 0.2, 0.2, 1.0];
            glEnable(GL_LIGHTING);									
            glMaterialfv(GL_FRONT, GL_AMBIENT, frontColor);			# Set ambient property frontColor.
            glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);			# Set diffuse property frontColor.
            glScaled(0.5,0.5,0.5);										# Reduce camera size by 1/2.
            glTranslated(1.1,1.1,0.0);									# Translate it (1.1, 1.1, 0.0).
            camModel.render()
            glPopMatrix();												# Call the matrix on stack. wld2cam[cameraIndex].matrix() in here.

def drawFrame(leng):
    glDisable(GL_LIGHTING);	# Lighting is not needed for drawing axis.
    glBegin(GL_LINES);		# Start drawing lines.
    glColor3d(1,0,0);		# color of x-axis is red.
    glVertex3d(0,0,0);			
    glVertex3d(leng,0,0);	# Draw line(x-axis) from (0,0,0) to (len, 0, 0). 
    glColor3d(0,1,0);		# color of y-axis is green.
    glVertex3d(0,0,0);			
    glVertex3d(0,leng,0);	# Draw line(y-axis) from (0,0,0) to (0, len, 0).
    glColor3d(0,0,1);		# color of z-axis is  blue.
    glVertex3d(0,0,0);
    glVertex3d(0,0,leng);	# Draw line(z-axis) from (0,0,0) - (0, 0, len).
    glEnd();			# End drawing lines.

#*********************************************************************************
# Draw 'cow' object.
#*********************************************************************************/
def drawCow(_cow2wld, drawBB):

    glPushMatrix();		# Push the current matrix of GL into stack. This is because the matrix of GL will be change while drawing cow.

    # The information about location of cow to be drawn is stored in cow2wld matrix.
    # (Project2 hint) If you change the value of the cow2wld matrix or the current matrix, cow would rotate or move.
    glMultMatrixd(_cow2wld.T)

    drawFrame(5);										# Draw x, y, and z axis.
    frontColor = [0.8, 0.2, 0.9, 1.0];
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT, GL_AMBIENT, frontColor);		# Set ambient property frontColor.
    glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);		# Set diffuse property frontColor.
    cowModel.render()	# Draw cow. 
    glDisable(GL_LIGHTING);
    if drawBB:
        glBegin(GL_LINES);
        glColor3d(1,1,1);
        cow=cowModel
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);

        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);

        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);


        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);

        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);
        glEnd();
    glPopMatrix();			# Pop the matrix in stack to GL. Change it the matrix before drawing cow.
def drawFloor():

    glDisable(GL_LIGHTING);

    # Set color of the floor.
    # Assign checker-patterned texture.
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, floorTexID );

    # Draw the floor. Match the texture's coordinates and the floor's coordinates resp. 
    nrep=4
    glBegin(GL_POLYGON);
    glTexCoord2d(0,0);
    glVertex3d(-12,-0.1,-12);		# Texture's (0,0) is bound to (-12,-0.1,-12).
    glTexCoord2d(nrep,0);
    glVertex3d( 12,-0.1,-12);		# Texture's (1,0) is bound to (12,-0.1,-12).
    glTexCoord2d(nrep,nrep);
    glVertex3d( 12,-0.1, 12);		# Texture's (1,1) is bound to (12,-0.1,12).
    glTexCoord2d(0,nrep);
    glVertex3d(-12,-0.1, 12);		# Texture's (0,1) is bound to (-12,-0.1,12).
    glEnd();

    glDisable(GL_TEXTURE_2D);	
    drawFrame(5);				# Draw x, y, and z axis.

def display():
    global cameraIndex, cow2wld, animating, placementStarted
    global isDrag, pickInfo, cursorOnCowBoundingBox
    global recordPointOnRelease

    glClearColor(0.8, 0.9, 0.9, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadMatrixd(wld2cam[cameraIndex].T)

    drawOtherCamera()
    drawFloor()

    if animating:
        # drawSplineTrack(controlPoints)

        animTime = glfw.get_time() - animStartTime
        segmentCount = len(controlPoints)
        totalDuration = LAP_COUNT * segmentCount * SEGMENT_DURATION

        if animTime >= totalDuration:
            # Finish at the beginning of the cyclic curve.
            position, _ = splinePositionAndTangent(controlPoints, 0, 0.0)

            initialYaw = -np.pi / 2.0
            cy = np.cos(initialYaw)
            sy = np.sin(initialYaw)

            cow2wld = np.eye(4)
            cow2wld[0:3, 0:3] = np.array([
                [ cy, 0.0, sy],
                [0.0, 1.0, 0.0],
                [-sy, 0.0, cy]
            ])
            setTranslation(cow2wld, position)

            controlPoints.clear()
            animating = False

            # Return to the cursor-following mode requested by the assignment.
            placementStarted = True
            recordPointOnRelease = False
            isDrag = 0
            cursorOnCowBoundingBox = False
            pickInfo = None
            window = glfw.get_current_context()
            if window is not None:
                setHorizontalPickAnchor(window)
        else:
            curveTime = animTime / SEGMENT_DURATION
            segmentIndex = int(curveTime) % segmentCount
            u = curveTime - int(curveTime)
            position, tangent = splinePositionAndTangent(
                controlPoints, segmentIndex, u
            )
            cow2wld = makeCowTransform(position, tangent)

        drawCow(cow2wld, False)
    else:
        # Previously fixed control points remain as duplicated cows.
        for pointTransform in controlPoints:
            drawCow(pointTransform, False)

        drawCow(
            cow2wld,
            cursorOnCowBoundingBox and not placementStarted
        )

    glFlush()

def reshape(window, w, h):
    width = w;
    height = h;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);            # Select The Projection Matrix
    glLoadIdentity();                       # Reset The Projection Matrix
    # Define perspective projection frustum
    aspect = width/(float)(height);
    gluPerspective(45, aspect, 1, 1024);
    matProjection=glGetDoublev(GL_PROJECTION_MATRIX).T
    glMatrixMode(GL_MODELVIEW);             # Select The Modelview Matrix
    glLoadIdentity();                       # Reset The Projection Matrix

def initialize(window):
    global cursorOnCowBoundingBox, floorTexID, cameraIndex, camModel, cow2wld, cowModel
    cursorOnCowBoundingBox=False;
    # Set up OpenGL state
    glShadeModel(GL_SMOOTH);         # Set Smooth Shading
    glEnable(GL_DEPTH_TEST);         # Enables Depth Testing
    glDepthFunc(GL_LEQUAL);          # The Type Of Depth Test To Do
    # Use perspective correct interpolation if available
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    # Initialize the matrix stacks
    #width, height = glfw.get_window_size(window) # incorrect on mac retina
    width, height =glfw.get_framebuffer_size(window)
    reshape(window, width, height);
    # Define lighting for the scene
    lightDirection   = [1.0, 1.0, 1.0, 0];
    ambientIntensity = [0.1, 0.1, 0.1, 1.0];
    lightIntensity   = [0.9, 0.9, 0.9, 1.0];
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientIntensity);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightIntensity);
    glLightfv(GL_LIGHT0, GL_POSITION, lightDirection);
    glEnable(GL_LIGHT0);

    # initialize floor
    im = open('bricks.bmp')
    try:
        ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGB", 0, -1)
    except SystemError:
        ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBX", 0, -1)

    # Make texture which is accessible through floorTexID. 
    floorTexID=glGenTextures( 1)
    glBindTexture(GL_TEXTURE_2D, floorTexID);		
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, ix, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    # initialize cow
    cowModel=OBJ.OBJrenderer("cow.obj")

    # initialize cow2wld matrix
    glPushMatrix();		        # Push the current matrix of GL into stack.
    glLoadIdentity();		        # Set the GL matrix Identity matrix.
    glTranslated(0,-cowModel.bbmin[1],-8);	# Set the location of cow.
    glRotated(-90, 0, 1, 0);		# Set the direction of cow. These information are stored in the matrix of GL.
    cow2wld=glGetDoublev(GL_MODELVIEW_MATRIX).T # convert column-major to row-major 
    glPopMatrix();			# Pop the matrix on stack to GL.


    # intialize camera model.
    camModel=OBJ.OBJrenderer("camera.obj")


    # initialize camera frame transforms.

    cameraCount=len(cameras)
    for i in range(cameraCount):
        # 'c' points the coordinate of i-th camera.
        c = cameras[i];										
        glPushMatrix();													# Push the current matrix of GL into stack.
        glLoadIdentity();												# Set the GL matrix Identity matrix.
        gluLookAt(c[0],c[1],c[2], c[3],c[4],c[5], c[6],c[7],c[8]);		# Setting the coordinate of camera.
        wld2cam.append(glGetDoublev(GL_MODELVIEW_MATRIX).T)
        glPopMatrix();													# Transfer the matrix that was pushed the stack to GL.
        cam2wld.append(np.linalg.inv(wld2cam[i]))
    cameraIndex = 0;

def onMouseButton(window, button, state, mods):
    global isDrag, placementStarted, recordPointOnRelease
    global animating, animStartTime, cursorOnCowBoundingBox

    x, y = glfw.get_cursor_pos(window)

    if button == glfw.MOUSE_BUTTON_LEFT:
        if animating:
            return

        if state == glfw.PRESS:
            # The very first click only picks the cow. Later clicks define points.
            if not placementStarted:
                if not cursorOnCowBoundingBox:
                    return
                placementStarted = True
                recordPointOnRelease = False
            else:
                recordPointOnRelease = True

            isDrag = V_DRAG
            startVerticalDrag(window, x, y)
            print("Left mouse down-click at %d %d" % (x, y))

        elif state == glfw.RELEASE and isDrag == V_DRAG:
            if recordPointOnRelease:
                controlPoints.append(cow2wld.copy())
                print("Control point %d/%d fixed" % (
                    len(controlPoints), CONTROL_POINT_COUNT
                ))

            recordPointOnRelease = False

            if len(controlPoints) == CONTROL_POINT_COUNT:
                animStartTime = glfw.get_time()
                animating = True
                isDrag = 0
                cursorOnCowBoundingBox = False
                print("Animation started")
            else:
                isDrag = H_DRAG
                setHorizontalPickAnchor(window)
                cursorOnCowBoundingBox = False
                print("Left mouse up - horizontal follow mode")

    elif button == glfw.MOUSE_BUTTON_RIGHT and state == glfw.PRESS:
        print("Right mouse click at (%d, %d)" % (x, y))

def onMouseDrag(window, x, y):
    global isDrag, cursorOnCowBoundingBox, pickInfo, cow2wld

    if animating:
        return

    if isDrag == V_DRAG:
        # Vertical movement: intersect the cursor ray with the fixed,
        # screen-facing drag plane and apply only its Y displacement.
        if verticalDragPlane is None or verticalDragStartConfig is None:
            return

        ray = screenCoordToRay(window, x, y)
        hit, t = ray.intersectsPlane(verticalDragPlane)
        if hit:
            currentPick = ray.getPoint(t)
            yOffset = currentPick[1] - verticalDragStartPick[1]

            T = np.eye(4)
            setTranslation(T, np.array((0.0, yOffset, 0.0)))
            cow2wld = T @ verticalDragStartConfig

    elif isDrag == H_DRAG:
        # Horizontal movement: keep the current height and move in the XZ plane.
        if pickInfo is None:
            setHorizontalPickAnchor(window)
            return

        ray = screenCoordToRay(window, x, y)
        pp = pickInfo
        plane = Plane(np.array((0.0, 1.0, 0.0)), pp.cowPickPosition)
        hit, t = ray.intersectsPlane(plane)

        if hit:
            currentPos = ray.getPoint(t)
            currentPos[1] = pp.cowPickPosition[1]

            T = np.eye(4)
            setTranslation(T, currentPos - pp.cowPickPosition)
            cow2wld = T @ pp.cowPickConfiguration

    else:
        # Idle mode: update bounding-box picking for the initial cow click.
        ray = screenCoordToRay(window, x, y)

        cow = cowModel
        bbmin = cow.bbmin
        bbmax = cow.bbmax
        planes = [
            makePlane(bbmin, bbmax, vector3(0, 1, 0)),
            makePlane(bbmin, bbmax, vector3(0, -1, 0)),
            makePlane(bbmin, bbmax, vector3(1, 0, 0)),
            makePlane(bbmin, bbmax, vector3(-1, 0, 0)),
            makePlane(bbmin, bbmax, vector3(0, 0, 1)),
            makePlane(bbmin, bbmax, vector3(0, 0, -1))
        ]

        hit, t = ray.intersectsPlanes(planes)
        cursorOnCowBoundingBox = hit

        if hit:
            cowPickPosition = ray.getPoint(t)
            cowPickLocalPos = transform(
                np.linalg.inv(cow2wld), cowPickPosition
            )
            pickInfo = PickInfo(
                t, cowPickPosition, cow2wld, cowPickLocalPos
            )
        else:
            pickInfo = None

def screenCoordToRay(window, x, y):
    width, height = glfw.get_window_size(window)

    matProjection=glGetDoublev(GL_PROJECTION_MATRIX).T
    matProjection=matProjection@wld2cam[cameraIndex]; # use @ for matrix mult.
    invMatProjection=np.linalg.inv(matProjection);
    # -1<=v.x<1 when 0<=x<width
    # -1<=v.y<1 when 0<=y<height
    vecAfterProjection =vector4(
            (float(x - 0))/(float(width))*2.0-1.0,
            -1*(((float(y - 0))/float(height))*2.0-1.0),
            -10)

    #std::cout<<"cowPosition in clip coordinate (NDC)"<<matProjection*cow2wld.getTranslation()<<std::endl;
	
    vecBeforeProjection=position3(invMatProjection@vecAfterProjection);

    rayOrigin=getTranslation(cam2wld[cameraIndex])
    return Ray(rayOrigin, normalize(vecBeforeProjection-rayOrigin))

def main():
    if not glfw.init():
        print ('GLFW initialization failed')
        sys.exit(-1)
    width = 800;
    height = 600;
    window = glfw.create_window(width, height, 'OpenGL Tutorial', None, None)
    if not window:
        glfw.terminate()
        sys.exit(-1)

    glfw.make_context_current(window)
    glfw.set_key_callback(window, onKeyPress)
    glfw.set_mouse_button_callback(window, onMouseButton)
    glfw.set_cursor_pos_callback(window, onMouseDrag)
    glfw.set_framebuffer_size_callback(window, reshape)
    glfw.swap_interval(1)

    initialize(window);						
    while not glfw.window_should_close(window):
        glfw.poll_events()
        display()

        glfw.swap_buffers(window)

    glfw.terminate()
if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*
# sample_python aims to allow seamless integration with lua.
# see examples below

import os
import sys
import pdb  # use pdb.set_trace() for debugging
import code # or use code.interact(local=dict(globals(), **locals()))  for debugging.
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image 
class Color:
    def __init__(self, R, G, B):
        self.color=np.array([R,G,B]).astype(np.float64)

    # Gamma corrects this color.
    # @param gamma the gamma value to use (2.2 is generally used).
    def gammaCorrect(self, gamma):
        inverseGamma = 1.0 / gamma;
        self.color=np.power(self.color, inverseGamma)

    def toUINT8(self):
        self.gammaCorrect(2.2)
        return (np.clip(self.color, 0,1)*255).astype(np.uint8)

class Camera:
    def __init__(self, element):
        self.element = element
        self.setCamera()

    def setCamera(self):
        # camera position
        self.viewPoint = np.array(self.element.findtext('viewPoint').split()).astype(np.float64)
        # normalize
        try:
            viewDir = np.array(self.element.findtext('viewDir').split()).astype(np.float64)
            self.viewDir = viewDir / np.linalg.norm(viewDir)
        except:
            self.viewDir=np.array([0,0,-1]).astype(np.float64)

        try:
            self.projNormal = np.array(self.element.findtext('projNormal').split()).astype(np.float64)
        except:
            self.viewProjNormal=-1*viewDir  # you can safely assume this. (no examples will use shifted perspective camera)

        # scene orientation
        try:
            self.viewUp = np.array(self.element.findtext('viewUp').split()).astype(np.float64)
        except:
            self.viewUp=np.array([0,1,0]).astype(np.float64)
        self.viewUp = np.cross(np.cross(self.viewDir, self.viewUp), self.viewDir)
        self.viewUp = self.viewUp / np.linalg.norm(self.viewUp)

        try:
            self.projDistance = np.array(self.element.findtext('projDistance').split()).astype(np.float64)[0]
        except:
            self.projDistance=1.0

        try:
            self.viewWidth = np.array(self.element.findtext('viewWidth').split()).astype(np.float64)[0]
        except:
            self.viewWidth=1.0

        try:
            self.viewHeight = np.array(self.element.findtext('viewHeight').split()).astype(np.float64)[0]
        except:
            self.viewHeight=1.0

class Shader:
    def __init__(self, element):
        self.element = element
        self.name = self.element.get('name')
        self.type = self.element.get('type')
        self.setShader()
    
    def setShader(self):
        self.diffuseColor = np.array(self.element.findtext('diffuseColor').split()).astype(np.float64)

        try:
            self.specularColor = np.array(self.element.findtext('specularColor').split()).astype(np.float64)
        except:
            self.specularColor = None

        try:
            self.exponent = np.array(self.element.findtext('exponent').split()).astype(np.float64)[0]
        except:
            self.exponent = None

    def applyShader(self, surface, v, icPoint, light):
        n = surface.getNormalVector(icPoint)
        i = light.intensity
        l = light.position - icPoint
        l = l / np.linalg.norm(l)
        if self.type == 'Phong':
            return self.applyPhong(-v, i, n, l)
        if self.type == 'Lambertian':
            return self.applyLambertian(i, n, l)
    
    def applyPhong(self, v, i, n, l):
        h = v + l
        normH = np.linalg.norm(h)
        if normH == 0.0:
            h = np.inf
        else: h = h / normH
        return ((self.specularColor * i) * (max(0, np.dot(n, h)) ** self.exponent)) + self.applyLambertian(i, n, l)

    def applyLambertian(self, i, n, l):
        return (self.diffuseColor * i) * max(0, np.dot(n, l))

class Light:
    def __init__(self, element):
        self.element = element
        self.setLight()

    def setLight(self):
        self.intensity=np.array([1,1,1]).astype(np.float64)
        self.position = np.array(self.element.findtext('position').split()).astype(np.float64)
        try:
            self.intensity = np.array(self.element.findtext('intensity').split()).astype(np.float64)
        except:
            pass

class Surface:
    def __init__(self, shader):
        self.shader = shader
    def rayIntersect(self, p, d): pass
    def getNormalVector(self, icPoint): pass

class Sphere(Surface):
    def __init__(self, shader, vcenter, radius):
        super().__init__(shader)
        self.vcenter = vcenter
        self.radius = radius

    def rayIntersect(self, p, d):
        p = p - self.vcenter
        dotProduct = np.dot(d, p)
        D = (dotProduct ** 2) - np.dot(p, p) + (self.radius ** 2)
        if D < 0: return None
        t1 = - dotProduct - np.sqrt(D)
        t2 = - dotProduct + np.sqrt(D)
        if t1 > 0: return t1
        if t2 > 0: return t2
        if t1 == 0.0 or t2 == 0.0: return 0.0
        return None

    def getNormalVector(self, icPoint):
        n = icPoint - self.vcenter
        return n / np.linalg.norm(n)

class Box(Surface):
    def __init__(self, shader, vmin, vmax):
        super().__init__(shader)
        self.vmin = vmin
        self.vmax = vmax

    def rayIntersect(self, p, d):
        tmin = np.finfo(np.float64).min
        tmax = np.finfo(np.float64).max

        for i in range(3):
            if d[i] == 0: pass
            tmin = max(tmin, min((self.vmin[i]-p[i])/d[i], (self.vmax[i]-p[i])/d[i]))
            tmax = min(tmax, max((self.vmin[i]-p[i])/d[i], (self.vmax[i]-p[i])/d[i]))

        if tmax < tmin: return None
        if tmin >= 0.0: return tmin
        if tmax >= 0.0: return tmax
        return None

    def getNormalVector(self, icPoint):
        eps = 1e-6
        if abs(icPoint[0] - self.vmin[0]) < eps: return np.array([-1.0, 0.0, 0.0])
        if abs(icPoint[0] - self.vmax[0]) < eps: return np.array([1.0, 0.0, 0.0])
        if abs(icPoint[1] - self.vmin[1]) < eps: return np.array([0.0, -1.0, 0.0])
        if abs(icPoint[1] - self.vmax[1]) < eps: return np.array([0.0, 1.0, 0.0])
        if abs(icPoint[2] - self.vmin[2]) < eps: return np.array([0.0, 0.0, -1.0])
        if abs(icPoint[2] - self.vmax[2]) < eps: return np.array([0.0, 0.0, 1.0])
    
        distances = [
            abs(icPoint[0] - self.vmin[0]),
            abs(icPoint[0] - self.vmax[0]),
            abs(icPoint[1] - self.vmin[1]),
            abs(icPoint[1] - self.vmax[1]),
            abs(icPoint[2] - self.vmin[2]),
            abs(icPoint[2] - self.vmax[2])
        ]
        face_indices = [
            [-1,0,0],
            [1,0,0],
            [0,-1,0],
            [0,1,0],
            [0,0,-1],
            [0,0,1]
        ]
        return np.array(face_indices[np.argmin(distances)], dtype=np.float64)

class Triangle(Surface):
    pass

class Scene:
    def __init__(self, tree, channels=3):
        self.tree = tree
        self.root = self.tree.getroot()

        self.cameras = []
        self.initCamera()

        self.channels = channels
        self.initImage(self.channels)

        self.shaders = {}
        self.initShader()

        self.lights = []
        self.initLight()

        self.surfaces = []
        self.initSurface()

    def initCamera(self):
        for c in self.root.findall('camera'):
            self.cameras.append(Camera(c))
    
    def initImage(self, channels):
        self.imgSize=np.array(self.root.findtext('image').split()).astype(np.int32)
        self.img = np.zeros((self.imgSize[1], self.imgSize[0], channels), dtype=np.uint8)
        self.img[:,:]=0

    def initShader(self):
        for s in self.root.findall('shader'):
            shader = Shader(s)
            self.shaders[shader.name] = shader

    def initLight(self):
        for l in self.root.findall('light'):
            self.lights.append(Light(l))

    def initSurface(self):
        for s in self.root.findall('surface'):
            type = s.get('type')
            shader = s.findall('shader')[0].get('ref')
            surface = None
            if type == 'Sphere':
                vcenter = np.array(s.findtext('center').split()).astype(np.float64)
                radius = np.array(s.findtext('radius').split()).astype(np.float64)[0]
                surface = Sphere(shader, vcenter, radius)
            elif type == 'Box':
                vmin = np.array(s.findtext('minPt').split()).astype(np.float64)
                vmax = np.array(s.findtext('maxPt').split()).astype(np.float64)
                surface = Box(shader, vmin, vmax)
            elif type == 'Triangle':
                surface = Triangle(shader)

            if surface:
                self.surfaces.append(surface)


    def calculateEyeRay(self, x, y, camera: Camera):
        width, height = camera.viewWidth, camera.viewHeight
        u = np.cross(-camera.viewUp, camera.viewDir) * (-width/2 + width*(0.5+x)/self.imgSize[0])
        v = -camera.viewUp * (-height/2 + height*(0.5+y)/self.imgSize[1])
        w = camera.viewDir * camera.projDistance
        r = u + v + w
        return r / np.linalg.norm(r)

    def rayTrace(self, camera: Camera):
        for y in np.arange(self.imgSize[1]):
            for x in np.arange(self.imgSize[0]):
                L = np.zeros(self.channels, dtype=np.float64)
                v = self.calculateEyeRay(x, y, camera)
                closestSurface = None
                found = False
                tmin = np.finfo(np.float64).max

                for surface in self.surfaces:
                    t = surface.rayIntersect(camera.viewPoint, v)
                    if t and t < tmin:
                        found = True
                        closestSurface = surface
                        tmin = t

                if not found:
                    continue

                icPoint = camera.viewPoint + tmin * v

                for light in self.lights:
                    blocking = False
                    u = icPoint - light.position
                    myT = closestSurface.rayIntersect(light.position, u / np.linalg.norm(u))
                    for surface in self.surfaces:
                        blockT = surface.rayIntersect(light.position, u / np.linalg.norm(u))
                        if blockT == None:
                            pass
                        elif blockT < myT:
                            blocking = True
                    if blocking:
                        continue
                    if closestSurface:
                        L += self.shaders[closestSurface.shader].applyShader(closestSurface, v, icPoint, light)

                self.img[y][x] = Color(L[0], L[1], L[2]).toUINT8()

    def createImage(self):
        if (len(self.cameras) == 1):
            self.rayTrace(camera=self.cameras[0])
            rawimg = Image.fromarray(self.img, 'RGB')
            if sys.argv[1]:
                rawimg.save(sys.argv[1]+'.png')
            else:
                rawimg.save('out.png')
        elif (len(self.cameras) > 1):
            for i in range(self.cameras.count):
                self.rayTrace(camera=self.cameras[i])
                rawimg = Image.fromarray(self.img, 'RGB')
                if sys.argv[1]:
                    rawimg.save(sys.argv[1]+str(i+1)+'.png')
                else:
                    rawimg.save('out'+str(i+1)+'.png')


def main():
    scene = Scene(ET.parse(sys.argv[1]))
    scene.createImage()
    
if __name__=="__main__":
    main()

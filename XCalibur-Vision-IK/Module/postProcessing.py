import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

def profileSmoothing(blade_profile, sigma):
    y = blade_profile[:,0,1]
    
    y_smooth = gaussian_filter1d(y, sigma=sigma, mode='nearest')
    
    blade_smooth = blade_profile.copy()
    blade_smooth[:,0,1] = y_smooth  # only y changes

    # re-add the edges of original
    blade_smooth[:2*sigma,0,1] = y[:2*sigma]
    blade_smooth[-2*sigma:,0,1] = y[-2*sigma:]
    
    return blade_smooth

def sparseArray(arr, n):
    if len(arr) == 0:
        return arr
    indices = np.arange(0, len(arr), n)
    if indices[-1] != len(arr) - 1:
        indices = np.append(indices, len(arr) - 1)
    return arr[indices]

def tangent(blade_profile, sigma=101):
    x = blade_profile[:,0,0].astype(float)  # <--- convert to float
    y = blade_profile[:,0,1].astype(float)
    
    dx = gaussian_filter1d(x, sigma=sigma, order=1, mode='reflect')
    dy = gaussian_filter1d(y, sigma=sigma, order=1, mode='reflect')
    
    tangents = np.vstack([dx, dy, np.zeros_like(dx, dtype=float)]).T  # make zeros float
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    
    return tangents

def bevelVectors(v_list, theta):
    result = []
    theta = math.radians(theta)

    for v in v_list:
        v1, v2, v3 = v

        b3 = math.sin(theta)

        A = 2.0 * (v2 * v3) / (v1 ** 2) * math.sin(theta)
        B = 1.0 + (v2 ** 2) / (v1 ** 2)
        C = (v3 ** 2) / (v1 ** 2) * (math.sin(theta) ** 2) - (math.cos(theta) ** 2)

        disc = A ** 2 - 4.0 * B * C
        if disc < 0:
            # If you want, you could skip this vector or set NaNs instead
            raise ValueError(f"No real solution for bevel vector: {v}")

        b2 = (-A + math.sqrt(disc)) / (2.0 * B * C)
        b1 = -(v2 / v1) * b2 - (v3 / v1) * b3
        b = np.array([b1, b2, b3], dtype=float)

        result.append(b)

    return np.array(result)

def normal(b_list,v_list):
    result = []
    for i in range(len(v_list)):
        b = b_list[i]
        v = v_list[i]

        c = np.cross(v,b)
        c = c/np.linalg.norm(c)
        result.append(c)
    return result

def to3D(smooth):
    # ensure shape is (n,2)
    smooth_2d = smooth.reshape(-1, 2)
    # add a zero column for z
    result = np.hstack([smooth_2d, np.zeros((smooth_2d.shape[0], 1))])
    return result

def swapXY(arr):
    for i in range(len(arr)):
        x = np.array([-arr[i][1], -arr[i][0], arr[i][2]])
        arr[i] = x
    return arr

def knifeGeo(blade_profile, theta):
    sigmaPos = 51
    sigmaTan = 101
    sampleRatio = 20

    smooth = profileSmoothing(blade_profile,sigmaPos)
    sparseSmooth = sparseArray(smooth, sampleRatio)
    tangents = tangent(smooth, sigmaTan)
    bevels = bevelVectors(tangents, theta)
    normals1 = normal(bevels,tangents)
    sparseTangents = sparseArray(tangents, sampleRatio)
    bevels = bevelVectors(sparseTangents, theta)
    smooth3D = to3D(sparseSmooth)
    normals1 = normal(bevels,sparseTangents)

    # need to change to global coords
    smooth3D = swapXY(smooth3D)
    normals1 = swapXY(normals1)
    
    return smooth3D, normals1

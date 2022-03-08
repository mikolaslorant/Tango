using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

public class CurvePlugin
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public unsafe struct VecControlPoint
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] startPoint;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] endPoint;
        public int controlPointNum;
        public IntPtr controlPointPtr;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public unsafe struct CurveValue
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] vec;    // x, y, z
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] quat;   // w, x, y, z
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] euler;  // x, y, z
    }

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern int CreateCurve();

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void RemoveCurve(int id);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void ResetCurve(int id);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void AppendVecKey(int id, double[] pos);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void AppendQuatKey(int id, double t, double[] q);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void AppendEulerKey(int id, double t, double[] angle);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern int InsertEulerKey(int id, double t, double[] angles);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern int InsertQuatKey(int id, double t, double[] q);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void DeleteVecKey(int id, int keyID);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void DeleteQuatKey(int id, int keyID);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void DeleteEulerKey(int id, int keyID);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void EditVecKey(int id, int keyID, double[] pos);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void EditQuatKey(int id, int keyID, double[] q);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void EditEulerKey(int id, int keyID, double[] angle);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void EditVecControlPoint(int id, int controlPointID, double[] pos);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetVecInterpolationType(int id, int type);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetQuatInterpolationType(int id, int type);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetEulerInterpolationType(int id, int type);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void GetCachedCurve(int id, ref int cachedPointNum, ref IntPtr cachedCurvePtr);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void GetControlPoints(int id, double[] startPoint, double[] endPoint, 
        ref int controlPointNum, ref IntPtr controlPointPtr);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern void GetValue(int id, double t, ref CurveValue curveValue);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern double GetVecDuration(int id);

    [DllImport("CurvePlugin", CallingConvention = CallingConvention.Cdecl)]
    public static extern int GetVecKeyNum(int id);
}

public class CurvePluginManager : MonoBehaviour
{

}

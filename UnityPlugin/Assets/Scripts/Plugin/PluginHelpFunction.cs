using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

public class PluginHelpFunction
{
    // Help functions

    #region CurvePlugin
    public static Vector3 DoubleArrayToVector3(double[] array)
    {
        return new Vector3((float)array[0], (float)array[1], (float)array[2]);
    }

    // Array(w, x, y, z)
    public static Quaternion DoubleArrayToQuaternion(double[] array)
    {
        return new Quaternion((float)array[1], (float)array[2], (float)array[3], (float)array[0]);
    }

    public static Vector3 FloatArrayToVector3(float[] array)
    {
        return new Vector3(array[0], array[1], array[2]);
    }

    // Array(w, x, y, z)
    public static Quaternion FloatArrayToQuaternion(float[] array)
    {
        return new Quaternion(array[1], array[2], array[3], array[0]);
    }

    public static double[] Vector3ToDoubleArray(Vector3 vec)
    {
        return new double[3] { vec.x, vec.y, vec.z };
    }

    public static double[] QuaternionToDoubleArray(Quaternion quat)
    {
        return new double[4] { quat.w, quat.x, quat.y, quat.z };
    }

    public static float[] Vector3ToFloatArray(Vector3 vec)
    {
        return new float[3] { vec.x, vec.y, vec.z };
    }

    public static float[] QuaternionToFloatArray(Quaternion quat)
    {
        return new float[4] { quat.w, quat.x, quat.y, quat.z };
    }

    public static Vector3[] DoubleArrayPointerToVector3Array(int size, IntPtr arrayPtr)
    {
        Vector3[] vectors = new Vector3[size];
        double[] doubleArray = new double[size * 3];
        Marshal.Copy(arrayPtr, doubleArray, 0, size * 3);
        for (int i = 0; i < size; ++i)
        {
            vectors[i] = new Vector3((float)doubleArray[3 * i],
                (float)doubleArray[3 * i + 1], (float)doubleArray[3 * i + 2]);
        }
        return vectors;
    }

    public static List<Vector3> DoubleArrayPointerToVector3List(int size, IntPtr arrayPtr)
    {
        List<Vector3> vectors = new List<Vector3>();
        double[] doubleArray = new double[size * 3];
        Marshal.Copy(arrayPtr, doubleArray, 0, size * 3);
        for (int i = 0; i < size; ++i)
        {
            vectors.Add(new Vector3((float)doubleArray[3 * i],
                (float)doubleArray[3 * i + 1], (float)doubleArray[3 * i + 2]));
        }
        return vectors;
    }

    #endregion
}

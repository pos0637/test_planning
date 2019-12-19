#include <string>
#include <limits.h>
#include <Python.h>
#include "python_bridge.h"

/**
 * @brief 初始化
 * 
 * @return int 返回值
 */
int Initialize()
{
    Py_Initialize();
    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *sys_path = PyObject_GetAttrString(sys, "path");
    PyObject *dir = PyUnicode_FromString(".");
    PyList_Append(sys_path, dir);

    return Py_IsInitialized();
}

int _ = Initialize();

/**
 * @brief 调用方法
 * 
 * @param moduleName 模块名称
 * @param methodName 方法名称
 * @return true 成功
 * @return false 失败
 */
bool Invoke(const std::string &moduleName, const std::string &methodName)
{
    // 加载模块
    PyObject *module = PyImport_ImportModule(moduleName.c_str());
    if (!module)
    {
        return false;
    }

    // 加载方法
    PyObject *pv = PyObject_GetAttrString(module, methodName.c_str());
    if (!pv || !PyCallable_Check(pv))
    {
        return false;
    }

    // 调用方法
    PyObject_CallObject(pv, NULL);

    return true;
}

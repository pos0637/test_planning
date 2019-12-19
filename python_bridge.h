#ifndef __PYTHON_BRIDGE_H__
#define __PYTHON_BRIDGE_H__

/**
 * @brief 调用方法
 * 
 * @param moduleName 模块名称
 * @param methodName 方法名称
 * @return true 成功
 * @return false 失败
 */
bool Invoke(const std::string &moduleName, const std::string &methodName);

#endif // __PYTHON_BRIDGE_H__

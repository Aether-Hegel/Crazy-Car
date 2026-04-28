# RT1064 Keil编译错误解决方案

## 问题描述
编译时出现错误：
```
ArmClang.exe: error: unsupported argument 'armasm' to option 'Wa,'
ArmClang.exe: error: unsupported argument '--diag_suppress=A1950W' to option 'Wa,'
```

## 问题原因
这些选项 `-Wa,armasm,--diag_suppress=A1950W` 是ARMASM（ARM汇编器）风格的选项，
与ArmClang编译器不兼容。它们可能来自：
1. Keil的全局配置
2. CMSIS组件包配置
3. 项目模板设置

## 解决方案

### 方法1：在Keil中手动修复（推荐）

1. **打开项目**
   - 使用Keil打开 `rt1064.uvprojx`

2. **检查Asm选项卡**
   - 点击 **Project** → **Options for Target**
   - 选择 **Asm** 选项卡
   - 检查 **Misc Controls** 是否有 `armasm` 或 `diag_suppress` 相关选项
   - 如果有，清除这些选项

3. **清除旧编译缓存**
   - 删除 `Objects` 目录中的所有文件
   - 重新编译项目

### 方法2：检查全局配置

1. **清除Keil全局设置**
   - 关闭Keil
   - 删除 `D:\Keil_v5\UV4\global.prop` 的备份或修改版本
   - 或在Keil中重置全局设置

2. **检查环境变量**
   - 右键"此电脑" → 属性 → 高级系统设置 → 环境变量
   - 检查是否有 `ARMCC_ASMFLAGS` 等环境变量

### 方法3：检查组件包

如果在项目中使用了CMSIS组件：
1. 在Keil中打开 **Project** → **Manage Project Items**
2. 检查 **Components, Environment, Books** 标签
3. 查看是否有组件包含不兼容的设置

## 临时解决方案

如果暂时无法解决问题，可以尝试：

1. **使用ARMASM代替ArmClang编译汇编文件**：
   - 在Asm选项卡中，禁用 "Use ARM Assembler" 或类似选项

2. **修改startup文件**：
   - 将 `startup_MIMXRT1064.S` 重命名为 `startup_MIMXRT1064.s`（小写）
   - 这可能会触发Keil使用不同的编译流程

3. **创建新的启动文件**：
   - 从NXP SDK下载兼容ArmClang的startup文件

## 验证修复

修复后，重新编译项目，应该看到类似输出：
```
compiling startup_MIMXRT1064.S...
```
而不是错误信息。

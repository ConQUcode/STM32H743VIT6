# AC6 + FreeRTOS 工程说明

## 0. 超短版操作清单

如果你只是想快速记住这套机制，先看这里：

1. CubeMX 重新生成后，可能会把 FreeRTOS 的 portable 层写回错误版本
2. 当前工程真正需要的是：
   `portable/GCC/ARM_CM7/r0p1`
3. 当前工程依赖 `fix_freertos_ac6.bat / fix_freertos_ac6.ps1` 修补 Keil 工程文件
4. 当前验证结果是：CubeMX 重新生成后，脚本可自动修补，工程可直接正常编译
5. 真正判断修补是否生效，要看：
   - `fix_freertos_ac6.log`
   - `MDK-ARM/VIT6.uvprojx`
   - `MDK-ARM/VIT6.uvoptx`

## 0.1 当前路径状态与可移植性说明

当前这套机制已经按“相对路径优先”的思路整理过，具体如下：

1. `fix_freertos_ac6.ps1` 与 `fix_freertos_ac6.bat`
   这两个脚本内部都按“脚本自身所在目录”定位工程文件。
   也就是说：
   - bat 通过 `%~dp0` 找到自身目录
   - ps1 通过 `$MyInvocation.MyCommand.Path` 找到自身目录
   - 再去定位 `MDK-ARM/VIT6.uvprojx`

   这一层是 **相对工程根目录的可移植写法**。

2. Keil 的 `Before Build/Rebuild`
   当前在 `MDK-ARM/VIT6.uvprojx` 中配置为：

   ```text
   cmd.exe /c "..\fix_freertos_ac6.bat"
   ```

   这是相对 `uvprojx` 所在目录 `MDK-ARM` 的写法。

3. CubeMX 的 `UAScriptAfterPath`
   当前在 `VIT6.ioc` 中配置为：

   ```text
   fix_freertos_ac6.bat
   ```

   这是相对 `.ioc` 所在工程根目录的写法。

### 当前结论

所以如果只回答“现在到底是绝对路径还是相对路径”，准确说法是：

- **当前已经改成相对路径方案**
- **脚本内部逻辑也是按自身目录解析的**
- **这套机制本身就是朝可移植性整理过的**

### 对可移植性的影响

这意味着：

- 如果只是把整个 `VIT6` 文件夹整体移动到别的电脑、别的磁盘、别的目录
- 只要内部目录结构不变
- 这套脚本和触发链路原则上仍然应该可以工作

### 实际建议

当前真正影响可移植性的，不是盘符或根目录变化，而是**工程内部相对结构变化**。

也就是说，下面这些关系最好不要随意改：

- `VIT6.ioc`
- `fix_freertos_ac6.bat`
- `fix_freertos_ac6.ps1`
- `MDK-ARM/VIT6.uvprojx`

所以现阶段的判断是：

**当前方案已经兼顾了可用性和可移植性，前提是保持现有目录结构不变。**

## 1. 文档用途

这份文档用于说明本工程在以下组合下为什么会出现 FreeRTOS 编译问题，以及当前是如何绕过并修复这些问题的：

- 芯片：`STM32H743`，内核为 `Cortex-M7`
- IDE：`Keil uVision`
- 编译器：`ARM Compiler 6 / armclang`
- 代码生成工具：`STM32CubeMX`
- RTOS：`FreeRTOS`

这不是业务代码问题，而是 **CubeMX 重新生成 Keil 工程后，FreeRTOS 移植层（portable layer）与当前编译器/内核组合不匹配** 导致的问题。

---

## 2. 问题背景

### 2.1 当前工程真正需要的 FreeRTOS 移植层

当前工程使用的是：

- `ARM Compiler 6`
- `Cortex-M7`

对这套组合，当前工程中实际使用的 FreeRTOS `port.c` 目标应为：

```text
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
```

### 2.2 CubeMX 重新生成后常见的错误状态

CubeMX 重新生成工程后，可能会把 Keil 工程文件中的 FreeRTOS portable 配置写回成不兼容版本，例如：

```text
Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F
```

这个版本对当前工程有两个明显问题：

1. 内核不对  
   当前工程是 `CM7`，而不是 `CM4F`

2. 语法/编译器风格不对  
   `RVDS/ARM_CM4F` 中包含较多旧式 RVDS/ARMCC 风格写法，例如：
   - `__forceinline`
   - `__asm void ...`
   - `PRESERVE8`

这些写法在 `armclang` 下会报大量错误。

### 2.3 典型报错现象

如果工程被 CubeMX 写回到错误的 portable 目录，常见症状包括：

- 日志中出现：
  `portable/RVDS/ARM_CM4F/port.c`
- 编译 `port.c` 时出现：
  - `This port can only be used when the project options are configured to enable hardware floating point support.`
  - `expected '(' after 'asm'`
  - `use of undeclared identifier 'PRESERVE8'`
  - `unknown type name '__weak'`

这些错误本质上都指向同一个问题：

**编译器、内核、FreeRTOS portable 版本三者不匹配。**

---

## 3. 当前采用的解决方案

为了避免每次手工修工程文件，当前工程引入了一套“自动修补”机制。

### 3.1 修补脚本

工程根目录下包含两个脚本：

- `fix_freertos_ac6.ps1`
- `fix_freertos_ac6.bat`

其中：

- `ps1` 是实际执行修补逻辑的脚本
- `bat` 是给 Keil / CubeMX 调用的包装器

### 3.2 脚本做了什么

脚本会自动检查并修补以下工程文件：

- `MDK-ARM/VIT6.uvprojx`
- `MDK-ARM/VIT6.uvoptx`

主要修补内容包括：

1. 将错误的 portable 路径替换掉，例如：
   - `portable/RVDS/ARM_CM4F`
   - `portable/RVDS/ARM_CM7/r0p1`

2. 统一替换为：

```text
portable/GCC/ARM_CM7/r0p1
```

3. 尝试删除或去重 Keil 工程里重复的 `port.c` 条目

这是因为 CubeMX 重新生成后，工程里有时不只是“路径写错”，还会把 `port.c` 挂两份，导致：

- 一个 `port.c` 指向正确版本
- 另一个 `port.c` 仍然指向旧版本

然后 Keil 会出现类似提示：

```text
object file renamed from 'port.o' to 'port_1.o'
```

这通常就说明工程里有重复的 `port.c`。

---

## 4. 触发链路

当前工程尝试了两条触发链路。

### 4.1 CubeMX 生成后触发

在 `VIT6.ioc` 中配置了：

```text
ProjectManager.UAScriptAfterPath=...
```

理论上，CubeMX 生成代码后会自动调用脚本。

### 4.2 Keil 编译前触发

在 `MDK-ARM/VIT6.uvprojx` 中配置了 `Before Build/Rebuild` 用户命令。

当前命令本质上是：

```bat
cmd.exe /c "C:\Users\11737\Desktop\VIT6\fix_freertos_ac6.bat"
```

也就是说，即使 CubeMX 那一条链路没有成功执行，只要在 Keil 中开始 Build/Rebuild，脚本也会先被执行一次。

---

## 5. 日志文件怎么看

脚本执行时会在工程根目录写日志：

```text
C:\Users\11737\Desktop\VIT6\fix_freertos_ac6.log
```

注意：

- **日志在工程根目录**
- **不在 `MDK-ARM` 目录里**

日志中如果出现类似内容：

```text
[日期 时间] run fix_freertos_ac6.bat
[日期 时间] exit code 0
```

说明脚本已经被触发，并且执行结束。

如果 `exit code` 不是 `0`，说明脚本运行时出现了错误，需要进一步看具体报错内容。

---

## 6. 当前验证结果

经过当前阶段的修正与验证，这套链路现在的预期行为已经是：

1. 在 CubeMX 中修改 `.ioc`
2. 重新生成工程
3. 自动触发修补脚本
4. Keil 中直接编译通过

也就是说，当前已经不再把“第一次编译报错、reload 后第二次正常”当作目标行为，而是把它视为前面调试阶段出现过的过渡现象。

当前更合理的判断标准是：

- 重新生成后，`fix_freertos_ac6.log` 有更新
- `uvprojx / uvoptx` 中 FreeRTOS `port.c` 指向 `portable/GCC/ARM_CM7/r0p1`
- Keil 可直接编译通过

只要满足这三点，就可以认为当前工程链路是稳定可用的。

---

## 7. 推荐使用流程

当前推荐流程如下：

1. 在 CubeMX 中修改 `.ioc`
2. 点击 `Generate Code`
3. 回到工程根目录，确认日志 `fix_freertos_ac6.log` 是否更新
4. 打开或切回 Keil
5. 直接编译

如果此时编译通过，则说明这套自动修补链路工作正常。

---

## 8. 快速排查方法

如果以后 FreeRTOS 错误又出现，按下面顺序检查：

### 8.1 先看日志有没有更新

看：

```text
C:\Users\11737\Desktop\VIT6\fix_freertos_ac6.log
```

如果没有新时间戳，说明脚本没有被触发。

### 8.2 看 Build Output 里是不是旧的 `port.c`

如果日志里或编译日志里仍出现：

```text
portable/RVDS/ARM_CM4F/port.c
```

说明修补没有真正生效，或者工程文件里还有残留/重复条目。

### 8.3 看 Keil 工程文件里最终路径

重点检查：

- `MDK-ARM/VIT6.uvprojx`
- `MDK-ARM/VIT6.uvoptx`

应只保留：

```text
portable/GCC/ARM_CM7/r0p1/port.c
```

不应再出现：

```text
portable/RVDS/ARM_CM4F/port.c
```

### 8.4 看是否有重复 `port.c`

如果工程里同时挂了两份 `port.c`，Keil 常会出现：

```text
object file renamed from 'port.o' to 'port_1.o'
```

这通常表示工程中同名源文件重复存在。

---

## 9. 当前已处理的附带问题

`Core/Src/freertos.c` 中已经在 `USER CODE` 区域加入：

```c
#include "Test.h"
```

这是为了让 `ChassisTask()` 的声明在 CubeMX 重新生成后仍然可见，避免出现类似：

```text
implicit declaration of function 'ChassisTask'
```

这样的 warning。

---

## 10. 当前结论

截至目前，这套机制的目标不是把流程做得绝对优雅，而是：

**保证在 CubeMX 会反复重新生成工程的前提下，AC6 + FreeRTOS 能继续稳定使用。**

所以当前这套方案可以这样理解：

- **脚本修的是工程配置**
- **不是业务代码**
- **当前验证结果是：重新生成后可直接恢复到 `GCC/ARM_CM7/r0p1` 并正常编译**
- **只要日志、工程文件路径、编译结果三者一致，就可以认为工程可用**

如果后面继续优化，优先级建议如下：

1. 保证脚本稳定触发
2. 保证修补后只剩一份正确的 `port.c`
3. 后续如果工程结构变化，再继续维护这套相对路径触发链路

从当前结果看，前两点已经基本达成。

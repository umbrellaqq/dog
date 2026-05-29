#include "iic.h"
esp_err_t iic_write_bytes(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

/*ESP32 内部的两个独立的硬件 I2C 控制器（I2C_NUM_0 和 I2C_NUM_1），分别控制MPU6050和pca9685*/
esp_err_t iic_init_all(void) {
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,/*选择模式，主机模式*/
        .sda_io_num = MPU_SDA_PIN,/*选择数据线*/
        .scl_io_num = MPU_SCL_PIN,/*选择时钟线*/
        .sda_pullup_en = GPIO_PULLUP_ENABLE,/*开启内部上拉*/
        .scl_pullup_en = GPIO_PULLUP_ENABLE,/*开启内部上拉*/
        .master.clk_speed = 400000,/*通信速率400kHz 叫做 Fast Mode（快速模式）。对于机器狗来说，这个配置极其关键。
        因为机器狗需要进行闭环自稳（算 PID），每秒钟可能要读取上百次 MPU6050 的姿态数据。
        如果用 100kHz，总线传输的时间太长，会严重拖慢主循环的计算频率，导致机器狗反应迟钝。*/
    };
    i2c_param_config(I2C_BUS_MPU, &conf0);/*参数配置，把参数交给具体的控制器I2C_BUS_MPU，让它按照这样的参数配置好*/
    i2c_driver_install(I2C_BUS_MPU, conf0.mode, 0, 0, 0);/*驱动安装，分配内存激活中断，第一个参数是选择控制器，第二个是选择主机或从机模式，
    第三个是接受缓冲区大小，第四个是发送缓冲区大小，最后一个是选择中断优先级。在这里填了两个 0，是因为 ESP32 是 主机（Master）。
    如果要理解为什么填 0，我们就得看看如果是 从机（Slave） 会发生什么。

主机的视角（ESP32）：
在 I2C 总线上，什么时候发数据、什么时候读数据，全由主机说了算。因为一切都在主机的计划之中，
ESP32 可以在需要发数据的时候直接把数据推给硬件，
不需要提前准备个“仓库”囤起来。所以，作为主机的 ESP32，接收和发送缓冲区大小都填 0，省下宝贵的 RAM 内存。

从机的视角（假设 ESP32 扮演从机）：
从机是个被动接客的服务员。它根本不知道主机（比如另一块板子）什么时候会突然发数据过来，或者突然索要数据。
为了防止主机发数据时从机来不及处理导致数据丢失，驱动程序必须在内存里划出一块“缓冲池（Buffer）”。主机突发过来的数据先扔进缓冲池，
从机的主程序等空闲了再去池子里捞数据。这时候，这两个参数可能就要填 256 甚至 512（字节）。
*/

    i2c_config_t conf1 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PCA_SDA_PIN,
        .scl_io_num = PCA_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_BUS_PCA, &conf1);
    return i2c_driver_install(I2C_BUS_PCA, conf1.mode, 0, 0, 0);
    /*

### 一、 表面逻辑：为什么要这样 `return`？（“击鼓传花”的艺术）

回头看一下这个函数的头顶：
`esp_err_t iic_init_all(void)`

你向编译器承诺过：**这个函数执行完后，一定会交出一个 `esp_err_t` 类型的报告（汇报工作是成功还是失败）**。

而乐鑫官方的 `i2c_driver_install` 函数，它在执行完之后，也会吐出一个 `esp_err_t` 类型的结果（比如修路成功返回 `ESP_OK`，内存不够返回 `ESP_ERR_NO_MEM`）。

所以，原作者在这里耍了一个“小聪明”，他用了**击鼓传花**的写法：
既然我 `iic_init_all` 需要向上级（比如 `main.c`）交一份报告，而 `i2c_driver_install` 刚好会给我一份报告，那我就直接把 `i2c_driver_install` 递给我的报告，原封不动地 `return` 给我的上级好了！

翻译成大白话就是：
* `main` 函数问 `iic_init_all`：“两条总线都修好了没？”
* `iic_init_all` 最后去修 PCA 总线，修完后直接把工头的原话回给了 `main`：“PCA 总线工头说：`ESP_OK`（搞定了）！”

---

### 二、 架构师视角：这种写法其实有个“致命漏洞”！

既然你已经到了给代码写注释的深度，我就必须用**高级工程师的标准**来挑刺了。

你仔细看这两行代码：
```c
// 第 1 条路：MPU 总线
i2c_driver_install(I2C_BUS_MPU, conf0.mode, 0, 0, 0); // ⚠️ 注意！这里没有处理返回值！

// 第 2 条路：PCA 总线
return i2c_driver_install(I2C_BUS_PCA, conf1.mode, 0, 0, 0); // 这里把返回值交上去了
```

**发现问题了吗？**
如果单片机在执行时，第一条 MPU 总线（0号公路）因为引脚冲突或者底层 BUG **初始化失败了**（它其实偷偷返回了 `ESP_FAIL`），但是代码根本没有去管它，而是继续往下跑。
紧接着，第二条 PCA 总线（1号公路）初始化成功了，返回了 `ESP_OK`。
最后，这个函数就会拿着第二条路的 `ESP_OK` 向上级汇报：“老板，全部初始化成功！”

这就好比你包工头接了修两段路的工程，第一段烂尾了，第二段修好了。老板问你咋样，你只把第二段的验收报告给老板看，老板以为全弄好了，结果机器狗一跑，读不到传感器数据，直接翻车。

### 三、 工业级重构：怎样写才是满分？

真正严谨的底层初始化代码，必须做到**“逢错必报，绝不隐瞒”**。你可以把你的代码改成这样：

```c
esp_err_t iic_init_all(void) {
    // ... 前面配好 conf0 ...
    i2c_param_config(I2C_BUS_MPU, &conf0);
    
    // 🌟 严谨写法：先把第一条路的结果存下来
    esp_err_t err = i2c_driver_install(I2C_BUS_MPU, conf0.mode, 0, 0, 0);
    
    // 🌟 如果第一条路修失败了，直接罢工，向上级报错，不再修第二条路
    if (err != ESP_OK) {
        return err; 
    }

    // ... 前面配好 conf1 ...
    i2c_param_config(I2C_BUS_PCA, &conf1);
    
    // 🌟 只有第一条路成功了，才会走到这里。
    // 这时候再把第二条路的结果 return 回去，才算真正的全盘掌控！
    return i2c_driver_install(I2C_BUS_PCA, conf1.mode, 0, 0, 0);
}
```

这就是 C 语言编程从“能跑就行”向“绝对健壮”蜕变的细节。把这个小雷排掉之后，你的 `iic.c` 就可以算是一个完美的底层驱动模块了！*/
}
/*port (Port)：端口。让人一看就知道这里该填总线编号。

addr (Address)：设备地址。特指 I2C 芯片在总线上的身份 ID。

reg (Register)：寄存器。这是硬件编程里最高频的词，代表芯片内部的存储单元。

data (Data)：数据。*/
esp_err_t iic_write_byte(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
/*
esp_err_t iic_read_bytes(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}*/
// iic.c
esp_err_t iic_read_bytes(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    i2c_master_start(cmd); // Repeated Start
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
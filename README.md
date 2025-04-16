## Настройка чистой системы
### 1. Установка IDE
Устанавливаем [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#overview) (необходим для настройки отладчика), [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html), [CLion](https://www.jetbrains.com/clion/).
### 2. Установка утилит
Выполняем установку утилит:
```
sudo apt install gcc-arm-none-eabi gdb-multiarch git openocd
```
### 3. Настройка отладчика  
**3.1** Переходим в директорию, где установлен STM32CubeIDE `/opt/st/stm32cubeide_*`, выполняем поиск файла "ST-LINK_gdbserver", копируем путь до этого файла.  
**3.2** Далее, в директории `/opt/st/stm32cubeide_*` ищем файл "cubeprogrammer", перходим `/tools/bin`, копируем путь.  
**3.3** Создаём в удобном месте bash-скрипт с содержимым:
```
#!/bin/bash
ST-LINK_gdbserver -p 61234 -l 1 -d -s -cp cubeprogrammer -m 0 -k

# Пример конкретного пути
#!/bin/bash
/opt/st/stm32cubeide_1.17.0/plugins/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.linux64_2.2.0.202409170845/tools/bin/ST-LINK_gdbserver -p 61234 -l 1 -d -s -cp /opt/st/stm32cubeide_1.17.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.0.202409170845/tools/bin -m 0 -k
```
После создания bash-скрипта необходимо выдать ему права на исполнение:
```
chmod +x путь_до_скрипта.sh
```
### 4. Настройка CLion и сборка проекта
**4.1** Клонируем репозиторий, можно использовать встроенную функцию в CLion или через терминал и добавить проект:
```
git clone https://github.com/13lyvnyv/SwitchLeds.git
```
**4.2** Переходим в `File -> Settings -> Build, Execution, Deployment -> Embenddend Development`, указываем путь к CubeMX (обычно находится ~/STM32CubeMX)  
**4.3** Далее, Переходим в `Run -> Edit Configurations` добавляем `Embedded GDB Server` и заполняем следующие поля:  

Executable: SwitchLeds.elf   
Upload executable: Always  
'target remote' args: localhost:61234  
GDB server: путь к ранее созданному bash-скрипту

**4.4** Настройка завершена. Проект готов к сборке и отладке.

### Примечания
**1.** Иногда при первом открытии проекта CLion не распознаёт CMakeLists.txt и не запускает сборку. В этом случае помогает перезапуск среды.  
**2.** Для работы STM32CubeIDE требуются библиотеки libtinfo5 и libncurses5:
```
wget http://archive.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.2-0ubuntu2.1_amd64.deb
wget http://archive.ubuntu.com/ubuntu/pool/universe/n/ncurses/libncurses5_6.2-0ubuntu2.1_amd64.deb
sudo dpkg -i libtinfo5_6.2-0ubuntu2.1_amd64.deb
sudo dpkg -i libncurses5_6.2-0ubuntu2.1_amd64.deb
```
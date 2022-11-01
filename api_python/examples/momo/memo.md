# socat 

create serial port
* move to directory
```
$ cd ~/kortex/api_python/examples/momo/
```
* using socat
```
socat -d -d pty,raw,echo=0,link=./SerialPortForPython pty,raw,echo=0,link=./SerialPortForMomo
```

# momo

connect serial port

* using momo.run
```
./momo --no-video-device --serial SerialPortForMomo,9600 test
```

* input 6-byte char at [momo p2p](http://127.0.0.1:8080/html/test.html)

```
$ xdg-open http://127.0.0.1:8080/html/test.html
```

# python

read received ~~6-byte~~ 8-byte character
px_py_pz_eux_euy_euz_gpos_rl

* run python script
```
python3 double_arm.py
```

* display serial port
```
python3 serial_recv.py
```

## KINOVA

* safe position
* World Positoin(m) x 0.44 y 0.19 z 0.45
* World Eular Rotation(deg) x 90.64 y -1.17 z 149.81

## 251

* Python code 
* cable : ip 192.168.2.13
* cable : ip 192.168.1.10
* without "with" method 

### concrete

* args to args1
* router to router1
* base to base1
* base_cyclic to base_cyclic1

### problem

* with to =
```
__init__, __enter__(), __exit__()
```
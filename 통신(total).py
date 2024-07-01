#serial communication
import tkinter as tk
from tkinter import ttk
from tkinter import scrolledtext
import serial.tools.list_ports
import threading
import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
matplotlib.use('TkAgg')

ports = serial.tools.list_ports.comports()
connected = []
id_list = []
trans = None
sPort = None
sVelo = None
state = None
targetMID = None #header = RMID
myMID = None #header = TMID
sID = None #ID
rpm = None
D1 = None
D2 = None
byCHK = None
parameterID = 130
DataNumber = 2
ringBF = []
limitBF = 10
cPut = 0  # write pointer
cGet = 0  # read pointer
ExSec = 0
Sec = 0
cRing = 0
PID4 = 4
PID_MAIN_DATA = 193
DATA1 = 1
DATA2 = 2
bystop = 130
stopD1 = 0
stopD2 = 0
PID_COMMAND = 10
CMD_MAIN_BC_ON = 5
CMD_MAIN_BC_OFF = 6
PID_TYPE = 205
velocity = 0
Yvelo = None
win = tk.Tk()
win.title("mdas")

for i in ports:
    connected.append(i.device)

for i in range (256):
    id_list.append(i)


def selPort(event):
    global sPort
    port_value.set(combobox1.get())
    sPort = port_value.get()
    print(sPort)
def selVelo(event):
    global sVelo
    velo_value.set(combobox2.get())
    sVelo = velo_value.get()
    print(sVelo)
def selID(event):
    global sID
    ID_value.set(combobox3.get())
    sID = int(ID_value.get())
def selRMID(event):
    global targetMID
    RMID_value.set(combobox4.get())
    sRMID = RMID_value.get()
    print(sRMID)
    if sRMID == "BLDC_CTR(183)":
        targetMID = 183
def selTMID(event):
    global myMID
    TMID_value.set(combobox5.get())
    sTMID = TMID_value.get()
    print(sTMID)
    if sTMID == "MDUI(184)":
        myMID = 184

port_value = tk.StringVar()
velo_value = tk.StringVar()
ID_value = tk.StringVar()
RMID_value = tk.StringVar()
TMID_value = tk.StringVar()
def btnConnectPort():
    global trans,sPort,sVelo
    if trans:
        trans.close()
    trans = serial.Serial(sPort,sVelo,timeout = 1) #포트에 연결
    button.config(text = "Connected")
    print('connected')
def calculate_byCHK():
    global state, targetMID, myMID, sID, parameterID, DataNumber, D1, D2, byCHK,stopD1,stopD2,bystop,PID_MAIN_DATA,DATA1,PID4,DATA2,CMD_MAIN_BC_ON,CMD_MAIN_BC_OFF,PID_COMMAND,PID_TYPE
    if state == 'run':
        sum = targetMID + myMID + sID + parameterID + DataNumber + D1 + D2
    if state == 'graph':
        sum = targetMID + myMID + sID + parameterID + DataNumber + D1 + D2
    if state == 'stop':
        sum = targetMID + myMID + sID + bystop +DATA2 + stopD1 + stopD2
    if state == 'pid4':
        sum = targetMID + myMID + sID + PID4 + DATA1 + PID_MAIN_DATA
    if state == 'MainBCOn':
        sum = targetMID + myMID + sID + PID_COMMAND + DATA1 + CMD_MAIN_BC_ON
    if state == 'MainBCOFF':
        sum = targetMID + myMID + sID + PID_COMMAND + DATA1 + CMD_MAIN_BC_OFF
    if state == 'controllerType':
        sum = targetMID + myMID + sID + PID4 + DATA1 + PID_TYPE
    byChkSend = sum & 0xFF  # 보내는 값
    byCHK = (~byChkSend + 1) & 0xFF  # 보내는 값
    return byCHK
def send_message(message):
    global trans,byCHK,ringBF,state
    byCHK = calculate_byCHK()
    for number in ringBF:
        trans.write(number.to_bytes(1,'big')) #바이트 형식 데이터 전송
    trans.write(byCHK.to_bytes(1,'big')) #바이트 형식 체크섬 전송
def getBF(): #링버퍼에서 꺼내기, 쓰레드로 항시 돌아감
    global limitBF, ringBF, cGet, cRing, Sec, ExSec, trans
    while True:
        if cRing > 0:
            send_message(ringBF[0])
            ringBF.pop(0)
            cGet += 1
            cRing -= 1

            if cGet >= limitBF:
                cGet = 0
def putBF(): #링버퍼에 집어넣기
    global limitBF, ringBF, cPut, cGet, cRing,targetMID ,myMID,sID,parameterID,DataNumber,D1,D2,state,PID4,PID_MAIN_DATA,DATA1,bystop,stopD1,stopD2,DATA2,CMD_MAIN_BC_ON,CMD_MAIN_BC_OFF,PID_COMMAND
    if state == 'run':
        data = [targetMID,myMID,sID,parameterID,DataNumber,D1,D2]
    if state == 'graph':
        data = [targetMID,myMID,sID,parameterID,DataNumber,D1,D2]
    if state == 'stop':
        data = [targetMID, myMID, sID, bystop,DATA2, stopD1, stopD2]
    if state == 'pid4':
        data = [targetMID,myMID,sID,PID4,DATA1,PID_MAIN_DATA]
    if state == 'MainBCOn':
        data = [targetMID,myMID,sID,PID_COMMAND,DATA1,CMD_MAIN_BC_ON]
    if state == 'MainBCOFF':
        data = [targetMID, myMID,sID,PID_COMMAND,DATA1,CMD_MAIN_BC_OFF]
    if state == 'controllerType':
        data = [targetMID , myMID , sID , PID4 , DATA1 , PID_TYPE]
    for i in range (len(data)):
        ringBF.append(data[i])
        cPut += 1
        cRing += 1
        if cPut >= limitBF:
            cPut = 0
def sendDisplay():
    global targetMID, myMID, sID, parameterID, DataNumber, D1, D2,byCHK,PID4,PID_MAIN_DATA,DATA1,bystop,stopD1,stopD2,DATA2,CMD_MAIN_BC_ON,CMD_MAIN_BC_OFF,PID_COMMAND
    if state == 'run':
        text_display3.insert(tk.END,f"{targetMID} {myMID} {sID} {parameterID} {DataNumber} {D1} {D2} {byCHK}" + '\n')
        text_display3.see(tk.END)
    if state == 'graph':
        text_display3.insert(tk.END, f"{targetMID} {myMID} {sID} {parameterID} {DataNumber} {D1} {D2} {byCHK}" + '\n')
        text_display3.see(tk.END)
    if state == 'stop':
        text_display3.insert(tk.END, f"{targetMID} {myMID} {sID} {bystop} {DATA2} {stopD1} {stopD2} {byCHK}" + '\n')
        text_display3.see(tk.END)
    if state == 'pid4':
        text_display3.insert(tk.END,f"{targetMID} {myMID} {sID} {PID4} {DATA1} {PID_MAIN_DATA} {byCHK}" + '\n')
        text_display3.see(tk.END)
    if state == 'MainBCOn':
        text_display3.insert(tk.END,f"{targetMID} {myMID} {sID} {PID_COMMAND} {DATA1} {CMD_MAIN_BC_ON} {byCHK}" + '\n')
        text_display3.see(tk.END)
    if state == 'MainBCOFF':
        text_display3.insert(tk.END,f"{targetMID} {myMID} {sID} {PID_COMMAND} {DATA1} {CMD_MAIN_BC_OFF} {byCHK}" + '\n')
        text_display3.see(tk.END)
    if state == 'controllerType':
        text_display3.insert(tk.END,f"{targetMID} {myMID} {sID} {PID4} {DATA1} {PID_TYPE} {byCHK}" + '\n')
        text_display3.see(tk.END)
def decimal_to_signed_binary(decimal_num):
    if decimal_num >= 0:
        return bin(decimal_num & 0xFFFF)[2:].zfill(16)  # 양수인 경우 하위 16비트만 반환
    else:
        complement = bin(abs(decimal_num) & 0xFFFF)[2:].zfill(16)  # 음수인 경우 2의 보수를 취함
        complement = ''.join('1' if bit == '0' else '0' for bit in complement)  # 1과 0을 바꿈
        complement = bin(int(complement, 2) + 1)[2:].zfill(16)  # 1을 더함
        return complement
def binary_to_decimal(binary_str):
    return int(binary_str, 2)
def split_msb_lsb(binary_str):
    msb = binary_str[:8]
    lsb = binary_str[8:]
    return msb, lsb


def savegraphValue():
    global data,targetMID, myMID, sID, parameterID, DataNumber,byCHK,state,limitBF, ringBF, cPut, cGet, cRing, D1, D2, PID4, PID_MAIN_DATA, DATA1, bystop, stopD1, stopD2, DATA2, CMD_MAIN_BC_ON, CMD_MAIN_BC_OFF, PID_COMMAND,byCHK,state,velocity
    state = 'graph'
    velo = int(text_entry.get())

    midVelo = int(velo/2)
    # x 값 범위 설정 (0부터 2π까지)
    x = np.linspace(0, 2 * np.pi, velo)
    # 초기 y 값 설정
    y = np.sin(x) * velo / 2 + midVelo

    # 그래프 초기 설정
    fig, ax = plt.subplots()
    line, = ax.plot(x, y)
    ax.set_ylim(0, velo)  # y축 범위 설정

    # x 축 눈금 설정
    plt.xticks([0, np.pi / 2, np.pi, 3 * np.pi / 2, 2 * np.pi],
               ['0', r'$\frac{\pi}{2}$', r'$\pi$', r'$\frac{3\pi}{2}$', r'$2\pi$'])

    # 그래프 업데이트 함수
    def update(frame):
        global D1,D2,byCHK
        # 새로운 y 값 계산
        new_y = np.abs(np.sin(x + frame * 0.1) * velo / 2 + midVelo)
        new_y_int = [int(value) for value in new_y]  # 각 요소를 정수로 변환
        line.set_ydata(new_y_int)
        binary = decimal_to_signed_binary(new_y_int[0])
        msb, lsb = split_msb_lsb(binary)
        D1 = binary_to_decimal(lsb) #int형
        D2 = binary_to_decimal(msb)
        byCHK = calculate_byCHK()
        putBF()

        return line,

    # 애니메이션 생성
    ani = FuncAnimation(fig, update, interval=100)

    plt.show()

def savePositiveValue():
    global D1,D2,byCHK,state,velocity
    state = 'run'
    entry_value = text_entry.get()
    positive = int(entry_value)
    binary = decimal_to_signed_binary(positive)
    msb,lsb = split_msb_lsb(binary)
    D1 = binary_to_decimal(lsb)
    D2 = binary_to_decimal(msb)
    byCHK = calculate_byCHK()
    sendDisplay()
    putBF()


def saveNegativeValue():
    global D1,D2,byCHK,state,velocity
    state = 'run'
    entry_value = text_entry.get()
    negative = -int(entry_value)
    velocity = negative
    binary = decimal_to_signed_binary(negative)
    msb, lsb = split_msb_lsb(binary)
    D1 = binary_to_decimal(lsb)
    D2 = binary_to_decimal(msb)
    byCHK = calculate_byCHK()
    sendDisplay()
    putBF()

def PID4F():
    global state, byCHK
    state = 'pid4'
    putBF()
    byCHK = calculate_byCHK()
    sendDisplay()
    response = trans.read(23)
    decimal_values = [int(byte) for byte in response]
    text_display2.insert(tk.END, " ".join(map(str, decimal_values)) + '\n')
def stop():
    global state, byCHK
    state = 'stop'
    putBF()
    byCHK = calculate_byCHK()
    sendDisplay()
    plt.close()

def MainBCON():
    response = trans.read(23)
    decimal_values = [int(byte) for byte in response]
    print(decimal_values)
    text_display2.insert(tk.END," ".join(map(str,decimal_values))+ '\n')
    text_display2.see(tk.END)

def stateON():
    global state,byCHK,ExSec
    state = 'MainBCOn'
    putBF()
    byCHK = calculate_byCHK()
    sendDisplay()

    def CallBack():
        global state, byCHK, ExSec
        while state == 'MainBCOn':
            now = datetime.datetime.now()
            Sec = now.microsecond//10000
            if Sec != ExSec: #0.1초에 한번씩 MAINbc값 받아오기
                MainBCON()
                ExSec = Sec
    bcOnThread = threading.Thread(target=CallBack,daemon = True)
    bcOnThread.start()



def MainBcOFF():
    global state,byCHK
    state = 'MainBCOFF'
    putBF()
    byCHK = calculate_byCHK()
    sendDisplay()

def contollerType():
    global state,byCHK
    state = 'controllerType'
    putBF()
    byCHK = calculate_byCHK()
    sendDisplay()
    response = trans.read(20)
    decimal_values = [int(byte) for byte in response]
    text_display2.insert(tk.END," ".join(map(str,decimal_values))+ '\n')
    text_display2.see(tk.END)

label_frame1 = tk.LabelFrame(win, text=" Comm. settings ")
label_frame1.grid(row=0, column=0, padx=10, pady=10, ipadx=5, ipady=5,sticky="nsew")

ttk.Label(label_frame1, text="Ports     ").grid(row=0, column=0, padx=5, pady=5,sticky="ew")
ttk.Label(label_frame1, text="ID       ").grid(row=0, column=1, padx=5, pady=5,sticky="ew")
ttk.Label(label_frame1, text="Bauderate          ").grid(row=2, column=0, padx=5, pady=5,sticky="ew")
ttk.Label(label_frame1, text="RMID(Target MID)").grid(row=4, column=0, padx=5, pady=5,sticky="ew")
ttk.Label(label_frame1, text="TMID(My MID)    ").grid(row=6, column=0, padx=5, pady=5,sticky="ew")

#port connect
combobox1 = ttk.Combobox(label_frame1, values=connected,textvariable=port_value, state="readonly", width=3)
combobox1.grid(row=1, column=0, padx=5, pady=5,sticky="nsew")
#id select
combobox3 = ttk.Combobox(label_frame1, values=id_list,textvariable=ID_value, state="readonly", width=3)
combobox3.grid(row=1, column=1, padx=5, pady=5,sticky="ew")
#Bauderate
combobox2 = ttk.Combobox(label_frame1, values=[9600, 19200, 38400],textvariable = velo_value, state="readonly", width=7)
combobox2.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
#RMID
combobox4 = ttk.Combobox(label_frame1, values=["BLDC_CTR(183)"],textvariable= RMID_value, state="readonly", width=7)
combobox4.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
#TMID
combobox5 = ttk.Combobox(label_frame1, values=["MDUI(184)"],textvariable=TMID_value, state="readonly", width=7)
combobox5.grid(row=7, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

combobox1.bind("<<ComboboxSelected>>", selPort)
combobox2.bind("<<ComboboxSelected>>", selVelo)
combobox3.bind("<<ComboboxSelected>>", selID)
combobox4.bind("<<ComboboxSelected>>", selRMID)
combobox5.bind("<<ComboboxSelected>>", selTMID)


# Button
button = ttk.Button(label_frame1, text="Connect", command=btnConnectPort, width=15)
button.grid(row=8, column=0, columnspan=2, sticky="ew", pady=10)

label_frame2 = tk.LabelFrame(win, text=" Filter ")
label_frame2.grid(row=1, column=0,columnspan=2, padx=10, pady=10, ipadx=5, ipady=5,sticky="nsew")

text_display2 = scrolledtext.ScrolledText(label_frame2, height=10, width=80)
text_display2.grid(row=0,rowspan=7,column=0,sticky="nsew")

button_MBON = ttk.Button(label_frame2,text = "Main BC ON",command = stateON)
button_MBON.grid(row = 0, column = 5, sticky="nsew")
button_MBOFF = ttk.Button(label_frame2,text = "Main BC OFF",command = MainBcOFF)
button_MBOFF.grid(row = 1, column = 5, sticky="nsew")
button_controllerTYPE = ttk.Button(label_frame2,text = "controller TYPE",command = contollerType)
button_controllerTYPE.grid(row = 2, column = 5, sticky="nsew")
label_frame3 = tk.LabelFrame(win)
label_frame3.grid(row=0, column=1, padx=10, pady=10, ipadx=5, ipady=5, sticky="nsew")

ttk.Label(label_frame3, text="Velocity(rpm)").grid(row=0, column=0, padx=5, pady=5)

button2 = ttk.Button(label_frame3, text="Run(-)", command=saveNegativeValue)
button2.grid(row=0, column=1, padx=5, pady=10)

text_entry = tk.Entry(label_frame3, width=10)
text_entry.grid(row=0, column=2, padx=5, pady=10)

button3 = ttk.Button(label_frame3, text="Run(+)", command=savePositiveValue)
button3.grid(row=0, column=3, padx=5, pady=10)

rungraphButton = ttk.Button(label_frame3,text = "graph",command = savegraphValue)
rungraphButton.grid(row = 0 , column = 4)

text_display3 = scrolledtext.ScrolledText(label_frame3, height=10, width=40)
text_display3.grid(row=1, column=0, columnspan=4, padx=10, pady=10, sticky="ew")

ttk.Label(label_frame3, text="Rx Counter").grid(row=3, column=0, padx=5, pady=5, sticky="ew")
rx_counter_label = ttk.Label(label_frame3)
rx_counter_text = tk.Text(label_frame3, height=1, width=5)
rx_counter_text.grid(row=3, column=1, padx=5, pady=5,sticky="ew")
button4 = ttk.Button(label_frame3, text="Rx Clear")
button4.grid(row=3, column=2, padx=5, pady=10,sticky="ew")

ttk.Label(label_frame3, text="Tx Counter").grid(row=4, column=0, padx=5, pady=5,sticky="ew")
rx_counter_label = ttk.Label(label_frame3)
rx_counter_text = tk.Text(label_frame3, height=1, width=5)
rx_counter_text.grid(row=4, column=1, padx=5, pady=5,sticky="ew")
button5 = ttk.Button(label_frame3, text="Tx Clear")
button5.grid(row=4, column=2, padx=5, pady=10,sticky="ew")
button6 = ttk.Button(label_frame3,text = "Main Data Req",command=PID4F)
button6.grid(row = 3,column= 3,padx=5, pady=10,sticky="ew")
button7 = ttk.Button(label_frame3,text = "Stop",command=stop)
button7.grid(row = 4,column= 3,padx=5, pady=10,sticky="ew")


getBFThread = threading.Thread(target = getBF)
getBFThread.start()
win.mainloop()
getBFThread.join()
trans.close()

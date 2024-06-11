import serial
import time

# Ganti COM dengan port serial yang sesuai di komputer Anda.

#Ardu1 untuk motor maju
ardu1 = serial.Serial('COM5', 115200) #UNO#
#Ardu2 untuk lengan dkk.
ardu2 = serial.Serial('COM12', 115200) #Mega
#esp untuk motor kanan kiri
esp1 = serial.Serial('COM9', 115200) #esp
time.sleep(2)  # Menunggu Arduino untuk reset

data = 0
buff = 0
bufff = 0
# data sampah, atur sesuai skenario
sampah =   ['B', 'D', 'H', 'I', 'G', 'F', 'J', 'A', 'E']
t4sampah = ['M', 'O', 'K', 'L', 'L', 'M', 'N', 'K', 'M']
# t4sampa= ['3', '7', '6', '4', '8', '10', '2', '1', '9'] SK COUNT



def maju(datanya): #ardu1
    data1 = datanya
    ardu1.write(data1.encode())

def gerakRL(datanya): #esp
    data2 = datanya
    esp1.write(data2.encode())

def geraklengan(datanya): #ardu2
    data1 = datanya
    ardu2.write(data1.encode())

#looping utama program
while True: 
    #robot posisi nyala
    #masukan data untuk start
    # start = input("masukan angka '1' untuk mulai: ")
    # if start == "1":
     start = int(input("masukan angka '1' untuk mulai/retry sampah ke?: "))
    if start >= 1:
        # robot bergerak maju
        retry = start - 1
        buff = retry
        bufff = retry
        maju("go")
        time.sleep(1)
        while True: 
            if ardu1.in_waiting > 0: #menunggu slave ardu mngirim data robot stop
                break
        print("Misi maju selesai")

    while True:
    #robot mencari sampah
        time.sleep(0.01)
        posisi_sam = sampah[buff]
        gerakRL(posisi_sam)
        time.sleep(1)
        b = esp1.in_waiting
        while True: 
            if esp1.in_waiting != b: #sampah ditemukan
                print("oke siap")
                break
        
        if 0 <= buff <= 1:
            geraklengan("A")
            print("gerak lengan Y")
            c = ardu2.in_waiting
            while True: 
                print("tunggu fdbck lengan Y")
                if ardu2.in_waiting != c:
                    break

        if 2 <= buff <= 7:
            geraklengan("C")
            c = ardu2.in_waiting
            while True: 
                print("tunggu fdbck lengan Y2")
                if ardu2.in_waiting != c:
                    break
                    
        if buff = 8:
            geraklengan("A")
            c = ardu2.in_waiting
            while True: 
                print("tunggu fdbck BUANG lengan Y")
                if ardu2.in_waiting != c:
                    break
        

        
        time.sleep(0.01)
        posisi_t4sampah = t4sampah[bufff]
        print("siap buang ke t4sampah")
        gerakRL(posisi_t4sampah)
        a = esp1.in_waiting
        while True: 
            if esp1.in_waiting != a:
                print("oke siap2")
                break
        
        if 0 <= buff <= 1:
            geraklengan("B")
            c = ardu2.in_waiting
            print("tunggu fdbck BUANG lengan Y")
            while True: 
                if ardu2.in_waiting != c:
                    break
        
        if 2 <= buff <= 7:
            geraklengan("D")
            c = ardu2.in_waiting
            while True: 
                print("tunggu fdbck BUANG lengan Y2")
                if ardu2.in_waiting != c:
                    break
        
        if buff == 8:
            geraklengan("A")
            c = ardu2.in_waiting
            while True: 
                print("tunggu fdbck BUANG lengan Y")
                if ardu2.in_waiting != c:
                    break

        buff+=1
        bufff+=1

        
        

        

    
    

    


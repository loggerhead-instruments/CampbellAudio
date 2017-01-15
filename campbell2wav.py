# converts a and b files from Campbell to wav
# start at byte 26 for each file

startByte = 32
path = "/w/loggerhead/CampbellAudio/"
fileA = "MannA201609202100"
fileB = "MannB201609202100"
outFile = "201609202100.wav"

fWav = open(outFile, 'wb')

f = open(path + fileA, 'rb')
header = f.read(32)
print(header)
nbytes = int(header[24:30])
print(nbytes)
data1 = f.read(nbytes)
#fWav.write(data)
f.close()

f = open(path + fileB, 'rb')
header = f.read(32)
print(header)
nbytes = int(header[24:30])
print(nbytes)
data2 = f.read(nbytes)
f.close()

fWav.write(data1 + data2)
fWav.close()

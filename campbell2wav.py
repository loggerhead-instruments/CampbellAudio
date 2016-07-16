# converts a and b files from Campbell to wav
# start at byte 26 for each file

startByte = 32
path = "/w/loggerhead/CampbellAudio/"
fileA = "MannA00000004"
fileB = "MannB00000004"
outFile = "Mann00000004.wav"

fWav = open(outFile, 'wb')

f = open(fileA, 'rb')
f.seek(startByte)
data = f.read()
fWav.write(data)
f.close()

f = open(fileB, 'rb')
f.seek(startByte)
data = f.read()
fWav.write(data)
f.close()
fWav.close()

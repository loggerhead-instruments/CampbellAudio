% Campbell Wav
% Combine to files into wav file

fileA = 'c:\w\loggerhead\CampbellAudio\MannA201609202100';
fileB = 'c:\w\loggerhead\CampbellAudio\MannB201609202100';
outFile = 'c:\w\loggerhead\CampbellAudio\201609202100.wav';

fid = fopen(fileA);
header = fread(fid, 32);
wavhead = fread(fid, 44);
dataA = fread(fid, 'int16');
fclose(fid);

fid = fopen(fileB);
header = fread(fid, 32);
dataB = fread(fid, 'int16');
fclose(fid);

dataA = dataA(1:length(dataA)-1); %trim last point which is N
dataB = dataB(1:length(dataA)-1); %trim last point which is N

data = [dataA; dataB];
wavwrite(data / 32767.0, 44100, outFile);

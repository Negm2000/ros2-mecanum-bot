import pyaudio
import math
import struct
import wave


class Event:
    def __init__(self):
        self.listeners = []

    def __iadd__(self, listener):
        """Shortcut for using += to add a listener."""
        self.listeners.append(listener)
        return self

    def notify(self, *args, **kwargs):
        for listener in self.listeners:
            listener(*args, **kwargs)


class SpeechRecorder():
    onRecording = Event()

    def __init__(self, threshold, filename='sound.wav', MaxStopTime=30, chunk=1024) -> None:
        self.chunk = chunk
        self.threshold = threshold
        self.all = []
        self.MaxStopTime = MaxStopTime
        self.FileName = filename

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16,
                                  channels=1,
                                  rate=16000,
                                  input=True,
                                  output=True,
                                  frames_per_buffer=chunk)

    def GetStream(self, chunk):
        return self.stream.read(chunk)

    def rms(self, frame):
        count = len(frame)/2
        format = "%dh" % (count)
        # short is 16 bit int
        shorts = struct.unpack(format, frame)

        sum_squares = 0.0
        for sample in shorts:
            n = sample * (1.0/32768.0)
            sum_squares += n*n
        # compute the rms
        rms = math.pow(sum_squares/count, 0.5)
        return rms * 1000

    def WriteSpeech(self, WriteData):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        wf = wave.open(self.FileName, 'w')
        wf.setnchannels(1)
        wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(WriteData)
        wf.close()

    def KeepRecord(self, LastBlock):
        self.all.append(LastBlock)
        stoptime = 0
        rmsValue = 0
        MaxStopTime = self.MaxStopTime
        while stoptime < MaxStopTime:
            data = self.GetStream(self.chunk)

            rmsValue = self.rms(data)
            if rmsValue < self.threshold:
                stoptime += 1
                self.onRecording.notify(False)
            else:
                self.onRecording.notify(True)
                stoptime = 0

            self.all.append(data)

        data = b''.join(self.all)
        print("write to File")
        self.WriteSpeech(data)

    def listen(self):
        print("waiting for Speech")
        silence = True
        while silence:
            input = self.GetStream(self.chunk)
            rms_value = self.rms(input)
            if (rms_value > self.threshold):
                self.onRecording.notify(True)
                silence = False
                LastBlock = input
                self.KeepRecord(LastBlock)
            else:
                self.onRecording.notify(False)

import speech_recognition as sr
import os
from tempfile import TemporaryFile
r = sr.Recognizer()
def input():
#while True:
    with sr.Microphone() as source:
        print("Speak Anything :")
        r.adjust_for_ambient_noise(source)      
        audio = r.listen(source)
        text=r.recognize_google(audio)
        print(text)
        return text



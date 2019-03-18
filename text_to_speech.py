from gtts import gTTS 
import pygame
import os
from tempfile import TemporaryFile

def output(a):
  print(a)
  l=gTTS(text=a, lang='en') 
  l.save("out.mp3")
  file = "out.mp3"
  pygame.init()
  pygame.mixer.init()
  pygame.mixer.music.load(file)
  pygame.mixer.music.play()



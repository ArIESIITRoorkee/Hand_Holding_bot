from chatterbot import ChatBot
from chatterbot.trainers import ChatterBotCorpusTrainer
import speech_to_text as st
import text_to_speech as tp
import serial
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
pin=18
GPIO.setup(pin, GPIO.IN)
ser=serial.Serial (port="/dev/ttyACM0",
baudrate=9600,
timeout=None,
write_timeout = 0)
ser.flushInput()

def tcb():
  chatbot = ChatBot('Ron Obvious')
  #chatbot.storage.drop()
  # Create a new trainer for the chatbot
  trainer = ChatterBotCorpusTrainer(chatbot)
  a=""
  # Train the chatbot based on the english corpus
  trainer.train("chatterbot.corpus.english.drop")
  message = st.input()


  reply = chatbot.get_response(message)
  if 'Aries' in str(message):
    a="1"
  if 'SDS' in str(message):
    a="2"
 if 'Music' in str(message):

    a="3"
  if 'Fine' in str(message):

    a="4"
  if 'Lift' in str(message):

    a="5"
  tp.output(str(reply))
  return a
  
if __name__=='__main__':
  tcb()







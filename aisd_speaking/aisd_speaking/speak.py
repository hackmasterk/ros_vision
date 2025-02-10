import io
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
from aisd_msgs.srv import Speak

import rclpy
from rclpy.node import Node



class SpeakService(Node):

    def __init__(self):
        super().__init__('speak')
        self.srv = self.create_service(Speak, 'speak', self.speak_callback)

    def speak_callback(self, request, response):
        #Create a temporary storage object f to store audio content in
        with io.BytesIO() as f:
            #Use google text speach to convert text to speach
            gTTS(text=request.words, lang='en').write_to_fp(f)
            #Position at the begining of the audio content to start reading it
            f.seek(0)
            #Load the audio object in mp3 format
            song = AudioSegment.from_file(f, format="mp3")
            #Play the song in speakers
            play(song)
            response.response = "OK"
            return response



def main():
    rclpy.init()

    speak = SpeakService()

    rclpy.spin(speak)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/home/user/anaconda3/bin/python
#-*- encoding: utf8 -*-
# [START speech_transcribe_streaming_mic]
from __future__ import division

import json, shlex, socket, subprocess, sys, threading
import roslib; roslib.load_manifest('gspeech')
import rospy
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Int8
import shlex,subprocess,os,io
from std_srvs.srv import *
from google.cloud import speech


import re
import sys

from google.cloud import speech
import pyaudio
from six.moves import queue
from living_lab_robot_perception.msg import ReceiveTargetAction, ReceiveTargetFeedback, ReceiveTargetResult

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms


class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


class OrderTargetSSTServer:
    def __init__(self):

        self.order_server = actionlib.SimpleActionServer('sst_order_received', ReceiveTargetAction, self.handle_request_order, False)
        # self.sub_detect = rospy.Subscriber('detected_object', Result, self.handle_detector_result)
        # self.server = actionlib.SimpleActionServer('qrcode_detect', ReceiveTargetAction, self.handle_request_detect, False)
        language_code = 'ko-KR'  # a BCP-47 language tag

        self.client = speech.SpeechClient()
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code)
        self.streaming_config = speech.StreamingRecognitionConfig(
            config=config,
            interim_results=True)

        self.order_server.start()
        rospy.loginfo('[%s] initialized...'%rospy.get_name())

        # print(self.result_orientation)

    def handle_request_order(self, goal):
        print("Request!!!")

        with MicrophoneStream(RATE, CHUNK) as stream:
            audio_generator = stream.generator()
            requests = (speech.StreamingRecognizeRequest(audio_content=content)
                        for content in audio_generator)

            responses = self.client.streaming_recognize(self.streaming_config, requests)

            # Now, put the transcription responses to use.
            received_order = self.listen_print_loop(responses)

            result = ReceiveTargetResult()
            success = True

            if success:
                result.result = True
                result.data = received_order
                self.order_server.set_succeeded(result)


    def listen_print_loop(self, responses):
        num_chars_printed = 0
        for response in responses:
            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            # Display interim results, but with a carriage return at the end of the
            # line, so subsequent lines will overwrite them.
            #
            # If the previous result was longer than this one, we need to print
            # some extra spaces to overwrite the previous result
            overwrite_chars = ' ' * (num_chars_printed - len(transcript))

            if not result.is_final:
                sys.stdout.write(transcript + overwrite_chars + '\r')
                sys.stdout.flush()

                num_chars_printed = len(transcript)

            else:
                print(transcript + overwrite_chars)
                stt_result = transcript + overwrite_chars

                if("컵" in stt_result):
                    return 'cup'
                elif("병" in stt_result):
                    return 'bottle'
                elif("우유" in stt_result):
                    return 'milk'
                elif("집" in stt_result):
                    return 'go_home'

                # Exit recognition if any of the transcribed phrases could be
                # one of our keywords.
                if re.search(r'\b(exit|quit)\b', transcript, re.I):
                    print('Exiting..')
                    break

                num_chars_printed = 0


def main():
    rospy.init_node('sst_order_target_server')
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.

    OrderTargetSSTServer()
    rospy.spin()

    # with MicrophoneStream(RATE, CHUNK) as stream:
    #     audio_generator = stream.generator()
    #     requests = (speech.StreamingRecognizeRequest(audio_content=content)
    #                 for content in audio_generator)

    #     responses = client.streaming_recognize(streaming_config, requests)

    #     # Now, put the transcription responses to use.
    #     listen_print_loop(responses)


if __name__ == '__main__':
    main()
# [END speech_transcribe_streaming_mic]
import  azure.cognitiveservices.speech as speechsdk  
import time
import os
from dotenv import load_dotenv
import json
from pal_interaction_msgs.msg import TtsAction, TtsGoal, AudioPlayerState
from std_msgs.msg import String
import rospy
import actionlib
from openai import OpenAI
from tiago_common.open_ai_utils import OpenAIUtils
# from tiago_common.src.tiago_common.open_ai_utils import OpenAIUtils


# Creates an instance of a speech config with specified subscription key and service region.  
# Replace with your own subscription key and service region (e.g., "westus").  
class STT:
    def __init__(self) -> None:
        self.isPlaying=None
        audio_state_topic = '/audio_player/state'
        rospy.Subscriber(audio_state_topic, AudioPlayerState, self.audio_callback)
        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.tts_client.wait_for_server()
        self.nav_com = rospy.Publisher(nav_topic, String, queue_size=1)

    
    def audio_callback(self, data):
        self.isPlaying =  data.isPlaying
        

    def send_tts(self, text):
            goal = TtsGoal()
            goal.rawtext.text = text
            goal.rawtext.lang_id = "en_GB"
            # Send the goal and wait
            self.tts_client.send_goal_and_wait(goal)

    def speech_recognize(self):
        if not self.isPlaying:
            speech_config = speechsdk.SpeechConfig(subscription='', region='eastus')
            device_index = 1
            
            #For Tiago
            audio_config = speechsdk.audio.AudioConfig(device_name=f"plughw:{device_index},0")
            
            #For local testing
            # audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
            speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)

            keyword = "hello"  # Replace with your desired keyword
            open_ai_util = OpenAIUtils(api_key='')
            

            print("Speak into your microphone.")
            result = speech_recognizer.recognize_once_async().get()
            if result.reason == speechsdk.ResultReason.RecognizedSpeech:
                print('RECOGNIZED: {}'.format(result.text))
                # if keyword.lower() in result.text.lower():
                print('RECOGNIZED: {}'.format(result.text))
                response = open_ai_util.chat_completion(result.text)
                res_txt = response.choices[0].message.content
                try: 
                    json_res = json.loads(res_txt)
                    print(json_res)
                    res_type = json_res["type"]
                    content = json_res["content"]
                    print(f"res_type: {res_type}\n content: {content}")
                    if res_type == 'navigation':
                        self.send_tts(f"Heading to {content}")
                        self.nav_com.publish(content)
                    elif res_type == 'chat':
                            self.send_tts(content)
                except Exception as e:
                    print("Error while parsing the response")

        elif result.reason == speechsdk.ResultReason.NoMatch:
            print("No speech could be recognized: {}".format(result.no_match_details))  

        


#FOR TIAGO
if __name__ == "__main__":
    nav_topic = "/navigation/command"
    try:
        rospy.init_node('tiago_stt')
        load_dotenv('stt.env')
        rate = rospy.Rate(1) # 1hz
        stt = STT()
        while not rospy.is_shutdown():
            # speech_recognize(tts_client, self.nav_com)
            if stt.isPlaying == False:
                stt.speech_recognize()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


import  azure.cognitiveservices.speech as speechsdk  
import time
import os
from dotenv import load_dotenv
import json
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import rospy
import actionlib
from openai import OpenAI
from tiago_common.open_ai_utils import OpenAIUtils
# from tiago_common.src.tiago_common.open_ai_utils import OpenAIUtils


# Creates an instance of a speech config with specified subscription key and service region.  
# Replace with your own subscription key and service region (e.g., "westus").  
def speech_recognize(stt_client = None):
    speech_config = speechsdk.SpeechConfig(subscription='', region='eastus')
    device_index = 1
    
    #For Tiago
    audio_config = speechsdk.audio.AudioConfig(device_name=f"plughw:{device_index},0")
    
    #For local testing
    # audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)

    keyword = "stop"  # Replace with your desired keyword
    open_ai_util = OpenAIUtils(api_key='')
        
    def send_tts(text, tts_client):
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        # Send the goal and wait
        tts_client.send_goal_and_wait(goal)
    
    while True:
        print("Speak into your microphone.")
        result = speech_recognizer.recognize_once_async().get()
        if result.reason == speechsdk.ResultReason.RecognizedSpeech:
            if keyword.lower() in result.text.lower():
                print('KEYWORD RECOGNIZED: {}'.format(result.text))
                # send_tts("Bye", stt_client)
                break
            else:
                print('RECOGNIZED: {}'.format(result.text))
                response = open_ai_util.chat_completion(result.text)
                res_txt = response.choices[0].message.content
                try: 
                    json_res = json.loads(res_txt)
                    res_type = json_res["type"]
                    content = json_res["content"]
                    print(f"res_type: {res_type}\n content: {content}")
                    if res_type == 'navigation':
                        send_tts(f"Heading to {content}", stt_client)
                    elif res_type == 'chat':
                         send_tts(content, stt_client)
                except Exception as e:
                    print("Error while parsing the response")

        elif result.reason == speechsdk.ResultReason.NoMatch:
            print("No speech could be recognized: {}".format(result.no_match_details))
        time.sleep(0.25)
        


#FOR TIAGO
if __name__ == "__main__":
    try:
        rospy.init_node('tiago_stt')
        load_dotenv('stt.env')
        # client for text to speech
        tts_client =  actionlib.SimpleActionClient('/tts', TtsAction)
        tts_client.wait_for_server()
        speech_recognize(tts_client)

    except rospy.ROSInterruptException:
        pass

# For local
# if __name__ == "__main__":
#     speech_recognize()
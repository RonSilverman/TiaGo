import  azure.cognitiveservices.speech as speechsdk  
import time
import os
from dotenv import load_dotenv
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import rospy
import actionlib
from openai import OpenAI
import openai

# Creates an instance of a speech config with specified subscription key and service region.  
# Replace with your own subscription key and service region (e.g., "westus").  
class OpenAIUtils():
    def __init__(self, model = "gpt-4o", max_token = 70, api_key = None ) -> None:
        # Load the .env file in the directory to get the API key for OpenAI
        load_dotenv()
        self.__client = OpenAI(api_key=api_key)
        # User can define the model and token to be used for the chat completion
        self.__model = model
        self.__max_token = max_token
    
    # Getter and setter for the model and max_token
    @property
    def model(self):
        return self.__model
    
    @model.setter
    def model(self, model):
        self.__model = model
        
    @property
    def max_token(self):
        return self.__max_token
    
    @max_token.setter
    def max_token(self, max_token):
        self.__max_token = max_token
    
    
    # Function to send a prompt to the OpenAI API and get the response, it can take the model name and max_token as input. Otherwise, it will use the default values defined in the constructor
    def completion(self, prompt, model = None, max_token = None):
        # Check if the user has defined the model and max_token
        if model is None:
            model = self.__model
        if max_token is None:
            max_token = self.__max_token
        # Send the prompt to the OpenAI completion API and get the response
        response = self.__client.completions.create(
            model=model,
            prompt=prompt,
            max_tokens=max_token
        )
        return response    
    
    def chat_completion(self, prompt, model = None, max_token = None):
        # Check if the user has defined the model and max_token
        if model is None:
            model = self.__model
        if max_token is None:
            max_token = self.__max_token
        # Send the prompt to the OpenAI completion API and get the response
        response = self.__client.chat.completions.create(
            model=model,
            max_tokens=max_token,
            messages=[
                {"role": "system", "content": "You are inside a mobile robot tiago who is a very helpful assistant who speaks less but speaks fluently and consice. Be straight yet be humble. Say thank you every time. Use australian slangs and australian english terms as frequently as possible. Your response must not be more than 4 seconds long"},
                {"role":"user",
                 "content":prompt}
            ]
        )
        return response     

     
            
def speech_recognize_continuous(stt_client):
    speech_config = speechsdk.SpeechConfig(subscription='', region='')
    device_index = 1
    audio_config = speechsdk.audio.AudioConfig(device_name=f"plughw:{device_index},0")

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
                send_tts("Bye", stt_client)
                break
            else:
                response = open_ai_util.chat_completion(result.text)
                res_txt = response.choices[0].message.content
                print(res_txt)
                send_tts(res_txt, stt_client)
                print('RECOGNIZED: {}'.format(result.text))

        elif result.reason == speechsdk.ResultReason.NoMatch:
            print("No speech could be recognized: {}".format(result.no_match_details))
        time.sleep(0.5)
        



if __name__ == "__main__":
    try:
        rospy.init_node('tiago_stt')
        load_dotenv('stt.env')
        # client for text to speech
        tts_client =  actionlib.SimpleActionClient('/tts', TtsAction)
        tts_client.wait_for_server()
        speech_recognize_continuous(tts_client)

    except rospy.ROSInterruptException:
        pass

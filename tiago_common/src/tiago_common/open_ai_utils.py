from dotenv import load_dotenv
from openai import OpenAI
import openai

# Purpose: To create a wrapper for the OpenAI API, so that the code is more readable
# and the code is more maintainable
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
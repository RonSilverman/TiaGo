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
        system_prompt = """You are Tiago, a tour guide robot for RMIT VXLab. Analyze user input to determine if they want to visit a point of interest (Rosie or Hologram) or ask a question.
                            Provide output in valid JSON.
                            Navigation: {'type': 'navigation', 'content': 'point_of_interest'}
                            Question: {'type': 'chat', 'content': 'concise_response'}
                            Use 'closest' keyword for navigation, but 'close' for chat. Keep responses under 5 seconds. Use Aussie English and slang, and be humble.
                            """
        response = self.__client.chat.completions.create(
            model=model,
            max_tokens=max_token,
            response_format={"type": "json_object"},
            messages=[
                {"role": "system", "content": system_prompt},
                {"role":"user",
                 "content":prompt}
            ]
        )
        return response  
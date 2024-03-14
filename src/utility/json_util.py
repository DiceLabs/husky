import json

READ_MODE = 'r'

def load_json_file(json_file: str):
    with open(json_file, READ_MODE) as file:
        return json.load(file)
    
def get_key_from_data(json_data: str, key: str):
    return json_data[key]

def get_key_from_file(json_file: str, key: str):
    json_data = load_json_file(json_file=json_file)
    return get_key_from_data(json_data=json_data, key=key)



class Keywords:
    def __init__(self):
        self.connect_request_keyword = "connect_request_"
        self.connect_response_keyword = "connect_response_"
        self.action_result_keyword = "action_result_"
        self.action_package_keyword = "action_package_"
        self.end_connection_keyword = "end_connection_"
        self.suspend_simulation_keyword = "suspend_simulation_"
        self.reset_simulation_keyword = "reset_simulation_"
        self.initial_request_keyword = "initial_request"
        self.initial_response_keyword = "initial_response"

    def add_id(self, id):
        self.connect_request_keyword += id
        self.connect_response_keyword += id
        self.action_result_keyword += id
        self.action_package_keyword += id
        self.end_connection_keyword += id
        self.suspend_simulation_keyword += id
        self.reset_simulation_keyword += id
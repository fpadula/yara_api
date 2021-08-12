from inputs import get_gamepad
import threading

class Xbox360Gamepad():
    def __init__(self, key_codes) -> None:
        self.update_keys = True
        self.keys = {}
        for code in key_codes:
            self.keys[code] = 0
        self.t = threading.Thread(target=self.update_key_states)
        self.t.start()

    def stop(self):
        self.update_keys = False
        self.t.join()
        # self.t.terminate()
        # self.t.

    def update_key_states(self):
        while(self.update_keys):
            events = get_gamepad()
            for event in events:
                if event.code in self.keys:
                    self.keys[event.code] = event.state
                    # print(event.ev_type, event.code, event.state)

    def read_key_state(self, key_code):
        return self.keys[key_code]

def main():
    """Just print out some event infomation when the gamepad is used."""
    while 1:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)


if __name__ == "__main__":
    main()
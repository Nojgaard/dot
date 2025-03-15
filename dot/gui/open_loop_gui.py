import dearpygui.dearpygui as dpg

class OpenLoopGui:
    def __init__(self):
        self._is_enabled = dpg.add_checkbox(label="Use open loop", default_value=False)

    def is_enabled(self):
        return dpg.get_value(self._is_enabled)
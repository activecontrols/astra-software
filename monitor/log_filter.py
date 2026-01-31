import os
import time
from platformio.public import DeviceMonitorFilterBase

class log_filter(DeviceMonitorFilterBase):
    NAME = "log_filter"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.f = None
        self.do_log = False
        # Ensure the directory exists
        if not os.path.exists("filter-logs"):
            os.makedirs("filter-logs")
        print("--- Log filter initialized ---")

    def rx(self, text):
        # Remove carriage returns/newlines for clean matching
        clean_text = text.strip()

        if clean_text == "<<< LOG BEGIN >>>":
            self.do_log = True
            return text

        elif self.do_log and self.f is None:
            # This line is treated as the filename suffix
            timestamp = int(time.time() * 1000)
            file_path = os.path.join("filter-logs", f"{timestamp}_{clean_text}.txt")
            self.f = open(file_path, "w", encoding="utf-8")
            return text

        elif clean_text == "<<< LOG END >>>":
            self.do_log = False
            if self.f:
                self.f.close()
                self.f = None
            return text

        elif self.do_log and self.f:
            self.f.write(text + "\n")
            self.f.flush() # Force write to disk

        return text

    def tx(self, text):
        return text
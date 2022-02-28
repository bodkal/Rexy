import lcddriver
from time import *
from gpiozero import CPUTemperature
import subprocess
import psutil


lcd = lcddriver.lcd()
lcd.lcd_clear()


# Calculate memory information

res = str(subprocess.check_output(['hostname', '-I'])).split(' ')[0].replace("b'", "")

while True:
     # Get cpu statistics
    cpu = psutil.cpu_percent()
    memory = psutil.virtual_memory().percent
    
    # Convert Bytes to MB (Bytes -> KB -> MB)
    #available = int(memory.available/1024.0/1024.0)
    #total = int(memory.total/1024.0/1024.0)

    lcd.lcd_display_string(f"CPU: {cpu}% {round(CPUTemperature().temperature,1)}c", 1)
    lcd.lcd_display_string(f"RAM: {memory} %" , 2)
    sleep(5)
    lcd.lcd_clear()
    lcd.lcd_display_string(f"User: rexy", 1)
    lcd.lcd_display_string(f"IP:{res}" , 2)
    sleep(5)
    lcd.lcd_clear()






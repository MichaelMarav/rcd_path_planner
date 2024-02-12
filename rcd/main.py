from rcd.ray_caster import RCD
from rcd.utilities import user_input_mode, print_red, print_green
import sys




def main():
    user_input = user_input_mode()
    if  user_input == 1: # Online Mode
        ray_caster = RCD(False, None)
        input("Press something to Exit")     

    elif (user_input == 2): 
        filename = str(input("Enter the name of the .ppm map file in maps/ folder (e.g. my_map.ppm) \n"))
        ray_caster = RCD(True, filename)
    else:
        print_red("Please enter a valid mode number. Exiting.")
        sys.exit()
        
    print("Would you like to save the map?")
    print_green("Exiting successfuly")

    return
    
if __name__ == "__main__":
    main()
    
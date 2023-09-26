from rcd.interactive_grid import interactive_grid_generator
from rcd import ray_caster
from rcd.utilities import user_input_mode 





def main():
    if user_input_mode() == 1: # Online Mode
        grid = interactive_grid_generator()
    else:                      # Offline Mode
        pass

if __name__ == "__main__":
    main()
    
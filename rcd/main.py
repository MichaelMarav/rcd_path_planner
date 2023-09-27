from rcd.interactive_grid import interactive_grid_generator
from rcd import ray_caster
from rcd.utilities import user_input_mode 





def main():
    if user_input_mode() == 1: # Online Mode
        grid_generator = interactive_grid_generator() # Draw grid and insert robot and goal poses 
        # print(grid_generator.grid)
        # print(grid_generator.robot_pos)
        # print(grid_generator.target_pos)
    else:                      # Offline Mode
        pass

if __name__ == "__main__":
    main()
    
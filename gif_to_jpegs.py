import os
from PIL import Image

def gif_to_jpeg(gif_path):
    # Open the GIF file
    gif = Image.open(gif_path)
    
    # Get the base name of the GIF file (without extension)
    base_name = os.path.splitext(os.path.basename(gif_path))[0]
    
    # Create a directory with the base name
    output_dir = base_name
    os.makedirs(output_dir, exist_ok=True)
    
    # Iterate through each frame in the GIF
    frame_number = 0
    try:
        while True:
            # Save the current frame as a JPEG file
            frame_path = os.path.join(output_dir, f"{base_name}_frame_{frame_number}.jpeg")
            gif.convert("RGB").save(frame_path, "JPEG")
            
            # Move to the next frame
            frame_number += 1
            gif.seek(frame_number)
    except EOFError:
        # End of frames
        pass

    print(f"Frames saved to {output_dir} directory.")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python gif_to_jpeg.py <path_to_gif>")
    else:
        gif_path = sys.argv[1]
        gif_to_jpeg(gif_path)

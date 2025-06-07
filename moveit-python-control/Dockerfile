FROM ros:melodic-ros-base

# Set the working directory
WORKDIR /moveit-python-control

# Copy the requirements file
COPY requirements.txt ./

# Install the required packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    && pip3 install --no-cache-dir -r requirements.txt

# Copy the source code into the container
COPY src/ ./src/

# Set the entry point for the container
CMD ["python3", "src/add_objects.py"]
# -*- coding: utf-8 -*-

# Team Id : eYRC-BV#2
# Author List : Shivam B. Waghela, Akshdeep Rungta, Anuj Singh, Siddharth Vyas
# Filename: Audio_Processing_with_Serial_Transfer.py
# Theme: Bothoven

############## Musical Note Identification and Serial Transfer ##############

# Importing necessary modules
import numpy as np  # for numerical calculations like fourier transform
import wave  # for reading audio file
import struct  # for decoding the audio
import serial # for serial communication
import time

############################## Initialization ##############################

window_size = 1000  # Size of window to be used for detecting silence
sampling_freq = 44100  # Sampling frequency of audio signal
note_freq_list = [("A0", 27.50), ("A1", 55.00), ("A2", 110.00), ("A3", 220.00), ("A4", 440.00), ("A5", 880.00),
                  ("A6", 1760.00), ("A7", 3520.00), ("A8", 7040.00), ("B0", 30.87), ("B1", 61.74), ("B2", 123.47),
                  ("B3", 246.94), ("B4", 493.88), ("B5", 987.77), ("B6", 1975.53), ("B7", 3951.07), ("B8", 7902.13),
                  ("C0", 16.35), ("C1", 32.70), ("C2", 65.41), ("C3", 130.81), ("C4", 261.63), ("C5", 523.25),
                  ("C6", 1046.50), ("C7", 2093.00), ("C8", 4186.01), ("D0", 18.35), ("D1", 36.71), ("D2", 73.42),
                  ("D3", 146.83), ("D4", 293.66), ("D5", 587.33), ("D6", 1174.66), ("D7", 2349.32), ("D8", 4698.63),
                  ("E0", 20.60), ("E1", 41.20), ("E2", 82.41), ("E3", 164.81), ("E4", 329.63), ("E5", 659.25),
                  ("E6", 1318.51), ("E7", 2637.02), ("E8", 5274.04), ("F0", 21.83), ("F1", 43.65), ("F2", 87.31),
                  ("F3", 174.61), ("F4", 349.23), ("F5", 698.46), ("F6", 1396.91), ("F7", 2793.83), ("F8", 5587.65),
                  ("G0", 24.50), ("G1", 49.00), ("G2", 98.00), ("G3", 196.00), ("G4", 392.00), ("G5", 783.99),
                  ("G6", 1567.98), ("G7", 3135.96), ("G8", 6271.93)]  # notes with their corresponding frequencies


############################# Implementation ###############################

def play(sound_file):
    '''
    :param sound_file: a single test audio file as input argument
    :return: Notes present in the audio file
    '''

    # Read Audio
    file_length, sound = read_audio(sound_file)

    # Compute mean square of amplitude of audio frames under the window for detecting silence
    mean_square_amp = []  # for storing the mean square amplitude result
    frame_count = 0
    mean_square_result = 0  # for temporarily storing partial result
    for i in range(file_length):
        frame_count += 1
        mean_square_result += sound[i] ** 2
        if (frame_count > window_size):
            mean_square_result /= window_size
            mean_square_amp.append(mean_square_result)
            frame_count = 0
            mean_square_result = 0

    # Detect silence
    silence_indices = []  # stores the index of a block of window_size which is silent
    for i in range(len(mean_square_amp)):
        if mean_square_amp[i] < 0.1:  # If amplitude mean-square value is less than threshold then silence is present
            silence_indices.append(i)  # store the index of the silence block of window_size

    # Find the note frequency
    identified_freq = []  # for storing identified note frequency

    if silence_indices[0] != 0:  # True if first block of window_size in sound is not silent
        start = 0
        end = silence_indices[0] * window_size
        # compute fast fourier transform to identify the note frequency
        freq_in_hertz = note_freq_identifier(sound, start, end)
        identified_freq.append(freq_in_hertz)

    for index, element in enumerate(silence_indices):
        if index < len(silence_indices) - 1:  # To prevent out of bound index error
            if silence_indices[index + 1] - element > 1:  # finding next note location
                start = element * window_size
                end = silence_indices[index + 1] * window_size
                # compute fast fourier transform to identify the note frequency
                freq_in_hertz = note_freq_identifier(sound, start, end)
                identified_freq.append(freq_in_hertz)

    # Identify notes from frequency by comparing it with the known frequencies of notes
    identified_notes = []  # for storing the identified notes
    length1 = len(note_freq_list)
    length2 = len(identified_freq)
    for i in range(length2):
        for j in range(length1):
            if abs(note_freq_list[j][1] - identified_freq[i]) < 5:
                identified_notes.append(note_freq_list[j][0])
    return identified_notes


def note_freq_identifier(sound, start, end):
    '''
    :param sound: contains the amplitude data of each frame of the audio file
    :param start: starting index of the note
    :param end: ending index of the note
    :return: frequency in hertz
    '''
    # calculate discrete fourier transfer
    fourier = np.fft.fft(sound[start:end])
    freq_array = np.fft.fftfreq(len(fourier))
    max_freq_index = np.argmax(np.abs(fourier))  # find the index of largest frequency
    freq = freq_array[max_freq_index]
    freq_in_hertz = abs(freq * sampling_freq)
    return freq_in_hertz


def read_audio(sound_file):
    '''
    :param sound_file: a single test audio file as input argument
    :return: file_length, sound
    '''
    file_length = sound_file.getnframes()
    sound = np.zeros(file_length)  # Returns a new array of given shape and type, filled with zeros
    for i in range(file_length):
        data = sound_file.readframes(1)  # Reads and returns 1 frame of audio, as a bytes object
        data = struct.unpack("<h", data)  # decoding: byte string is mapped to a 2 bytes integer
        sound[i] = int(data[0])
    sound = np.divide(sound, float(2 ** 15))  # Normalize data in range -1 to 1 as the data is 2 bytes long
    # Now the sound contains the amplitude data of each frame of the audio file
    return file_length, sound

############################## Serial Transfer ################################

def serial_transfer():
    ser = serial.Serial("COM4", 9600)   # open serial port that Firebird V is using
    Notes = ['C6', 'C6', 'F7', 'A7', 'B7', 'C8', 'B8', 'E6', 'F6', 'B7', 'G6'] # given Notes
    MNP = ['4', '8', '22', '5', '12', '31', '28', '32', '15', '33', '24'] # given corresponding MNPs

    
    # output the appropriate MNP for the corresponding note from identified_notes
    MNP_seq = []
    for note in identified_notes:
        for i in range(len(Notes)):
            if Notes[i] == note:
                break
        MNP_seq.append(MNP[i])
        Notes.pop(i)
        MNP.pop(i)

    print(MNP_seq)
    # serially tranfer the MNP_list to the Firebird V robot
    for c in display_string:
        ser.write(c)
        time.sleep(1)
    ser.close()

############################## Testing Audio File #############################
identified_notes = ['C8', 'G6', 'B7', 'C6', 'C6']
if __name__ == "__main__":
    # code for checking output for all audio
    file_name = "/path/to/audio/file/audio_1.wav"
    sound_file = wave.open(file_name)
    identified_notes = play(sound_file)
    print("Notes in Audio = "),
    print(identified_notes)
    serial_transfer()
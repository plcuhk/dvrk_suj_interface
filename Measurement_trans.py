def hex2dec(hex_num):
    hex_dig_num = len(hex_num)
    dec_num = 0
    hex_dig_list = []
    
    for i in range(0, 10):
        hex_dig_list.append(str(i))
    for i in "ABCDEF":
        hex_dig_list.append(i)

    hex_dec_dict = dict(zip(hex_dig_list, range(0, 16)))
    hex_dig_lower_dict = dict(zip("abcdef", range(10, 16)))
    hex_dec_dict = {**hex_dec_dict, **hex_dig_lower_dict}
    # print(hex_dec_dict)

    for i, j in reversed(list(enumerate(hex_num))):
        dec_num += hex_dec_dict[j] * pow(16, hex_dig_num - i - 1)
    return dec_num

if __name__ == "__main__":
    full_range = hex2dec("FFFFFF")

    while True:
        measurement = input("Please input the measurement: ")
        dec_num = hex2dec(measurement)
        voltage = dec_num / full_range * 3.3
        print("voltage is %f" %voltage)

        switch = input("For stop the program, press n/N: ")
        if switch == "n" or switch == "N":
            break

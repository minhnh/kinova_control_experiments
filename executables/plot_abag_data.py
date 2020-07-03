#!/usr/bin/env python3
import argparse
import csv
from enum import Enum
import os
from matplotlib import pyplot as plt
import numpy as np


class ColumnsDesired(Enum):
    POS_X = 0
    POS_Y = 1
    POS_Z = 2

    def get_column(name):
        if name == 'pos_x':
            return ColumnsDesired.POS_X.value
        elif name == 'pos_y':
            return ColumnsDesired.POS_Y.value
        elif name == 'pos_z':
            return ColumnsDesired.POS_Z.value
        else:
            raise ValueError("invalid column name: " + name)


class Columns(Enum):
    TIME_POINT = 0
    ERROR = 1
    SIGNED_ERR = 2
    BIAS = 3
    GAIN = 4
    FILTERED_ERR = 5
    COMMAND = 6
    MEASURED = 7


def get_desired_value(desired_file, value_name, delimiter=','):
    with open(desired_file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=delimiter)
        headers = next(csv_reader)
        values = next(csv_reader)
    index = ColumnsDesired.get_column(value_name)

    return headers[index], float(values[index])


def read_csv_data(data_file, delimiter=','):
    data = {}
    with open(data_file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=delimiter)
        headers = next(csv_reader)
        for head in headers:
            data[head] = []
        for row in csv_reader:
            # assuming each row has the same number of items as headers
            for i, head in enumerate(headers):
                data[head].append(float(row[i]))
    return data, headers


def plot_csv_data(csv_file_path, desired_file, ctrl_dimension):
    if not os.path.isfile(csv_file_path):
        raise ValueError("CSV file '{}' does not exist".format(csv_file_path))

    data, headers = read_csv_data(csv_file_path)
    desired_val_name, desired_val = get_desired_value(desired_file, ctrl_dimension)
    print("desired: " + desired_val_name + " = ", desired_val)

    num_data_points = len(data[headers[Columns.TIME_POINT.value]])
    print("num data points: ", num_data_points)

    # Create two sub-plots sharing y axis
    fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, sharex=True)
    fig.set_figwidth(12)
    fig.set_figheight(8)

    # convert from nano to milliseconds
    data[headers[Columns.TIME_POINT.value]] = np.divide(data[headers[Columns.TIME_POINT.value]], 1000000.)
    gain_times_sign_err = np.multiply(data[headers[Columns.SIGNED_ERR.value]], data[headers[Columns.GAIN.value]])

    ax1.axhline(y=desired_val, label="desired " + desired_val_name, c='y')
    ax1.plot(data[headers[Columns.TIME_POINT.value]], data[headers[Columns.MEASURED.value]],
             label=headers[Columns.MEASURED.value], c='m')
    ax1.set(ylabel='Desired vs ' + headers[Columns.MEASURED.value])
    ax1.legend(loc=4)

    ax2.axhline(c='lightgray')
    ax2.plot(data[headers[Columns.TIME_POINT.value]], data[headers[Columns.FILTERED_ERR.value]])
    ax2.set(ylabel=headers[Columns.FILTERED_ERR.value])

    ax3.axhline(c='lightgray')
    ax3.plot(data[headers[Columns.TIME_POINT.value]], gain_times_sign_err, label='gain * signed err', c='r')
    ax3.plot(data[headers[Columns.TIME_POINT.value]], data[headers[Columns.COMMAND.value]], label='command', c='b')
    ax3.plot(data[headers[Columns.TIME_POINT.value]], data[headers[Columns.BIAS.value]], label='bias', c='g')
    ax3.legend(loc=3)
    ax3.set(ylabel='bias gain command')

    ax4.axhline(c='lightgray')
    ax4.plot(data[headers[Columns.TIME_POINT.value]], data[headers[Columns.BIAS.value]], c='g')
    ax4.set(ylabel=headers[Columns.BIAS.value])

    ax5.axhline(c='lightgray')
    ax5.plot(data[headers[Columns.TIME_POINT.value]], gain_times_sign_err, c='r')
    ax5.set(ylabel='gain * signed err')

    ax6.axhline(c='lightgray')
    ax6.plot(data[headers[Columns.TIME_POINT.value]], data[headers[Columns.COMMAND.value]], c='b')
    ax6.set(xlabel='time (ms)', ylabel=headers[Columns.COMMAND.value])

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="simple script to plot ABAG control data")
    parser.add_argument("csv_file", help="CSV file containing ABAG control data")
    parser.add_argument("desired_file", help="CSV file containing desired values")
    parser.add_argument("ctrl_dimension", choices=['pos_x', 'pos_y', 'pos_z'],
                        help="specify which dimension is being controlled in data")
    args = parser.parse_args()
    try:
        plot_csv_data(args.csv_file, args.desired_file, args.ctrl_dimension)
    except ValueError as e:
        print(e)

#!/usr/bin/env python3
# Copyright 2020 Carlos San Vicente
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description='Plot topic statistics metrics')
    parser.add_argument('filename', metavar='filename')
    parser.add_argument('-s', '--show', help='Show plots', action='store_true')
    parser.add_argument('-t', '--topicname', metavar='topicname', default=None)
    parser.add_argument('-m', '--metric', metavar='metric', default='message_period',
                        help='message_period or message_age')
    parser.add_argument('-o', '--outfile',
                        help='Name of file to write plot output')
    args = parser.parse_args()
    filename = args.filename
    topicname = args.topicname
    show = args.show
    metric = args.metric
    outfile = filename + '_plot'
    if args.outfile is not None:
        outfile = args.outfile
    else:
        if topicname:
            outfile = f"{topicname}_{metric}"
        else:
            outfile = f"topic_statistics_{metric}"

    col_names=["measurement_source_name",
               "metrics_source",
               "unit",
               "window_start_sec",
               "window_start_nsec",
               "window_stop_sec",
               "window_stop_nsec",
               "data_type1",
               "average",
               "data_type2",
               "minimum",
               "data_type3",
               "maximum",
               "data_type4",
               "standard_deviation",
               "data_type5",
               "sample_count"
               ]

    df_raw = pd.read_csv(filename, delimiter=',', names=col_names)

    # Convert window timestamps to relative time in seconds
    starttime = df_raw.loc[:, 'window_stop_sec'].iloc[0] + \
                df_raw.loc[:, 'window_stop_sec'].iloc[0] / 10E9
    df_raw['time'] = df_raw['window_stop_sec'].astype(float) + \
                     df_raw['window_stop_nsec'].astype(float) / 10E9 - starttime

    # Filter topic name and metrics
    if topicname:
        df = df_raw.loc[(df_raw.metrics_source == metric)
                        & (df_raw.measurement_source_name == topicname)]
    else:
        df = df_raw.loc[(df_raw.metrics_source == metric)]

    # Get metric units
    unit = df['unit'].iloc[0]

    # Plot topic statistic metrics
    plt.figure()
    ax = plt.gca()
    df.plot(kind='line', x='time', y='average', ax=ax)
    df.plot(kind='line', x='time', y='minimum', ax=ax)
    df.plot(kind='line', x='time', y='maximum', ax=ax)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(f"{metric} ({unit} )")
    plt.title(f"{metric}  vs. time")
    if show:
        plt.show()
    plt.savefig(f"{outfile}.svg")


if __name__ == '__main__':
    main()
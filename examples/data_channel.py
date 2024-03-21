import rbpodo as rb


def _main():
    data_channel = rb.CobotData("10.0.2.7")
    data = data_channel.request_data(1.0)
    print(data.sdata.tcp_ref)


if __name__ == "__main__":
    _main()

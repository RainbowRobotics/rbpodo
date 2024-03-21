import os
import csv
import argparse

header = """#pragma once

#include <map>
#include <string>

namespace rb::podo {

struct Message {
  std::string ko;
  std::string en;
};

inline std::map<int, Message> ErrorCodeMessage = {
"""

footer = """
};

}  // namespace rb::podo"""


def convert_ec(filename, output=None):
    if output is None:
        output = os.path.splitext(filename)[0] + '.hpp'

    assert os.path.isfile(filename)

    with open(filename, "r") as csvfile, open(output, "w") as outfile:
        ecm_reader = csv.reader(csvfile, delimiter='\t')

        items = []
        for row in ecm_reader:
            items.append(f"{{ {row[0]}, {{R\"({row[1]})\", R\"({row[2]})\"}} }}")

        outfile.write(header)
        outfile.write(",\n".join(items))
        outfile.write(footer)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Convert ErrorCode to C++ header")
    parser.add_argument('filename')
    parser.add_argument('-o', '--output')

    args = parser.parse_args()
    convert_ec(args.filename, output=args.output)

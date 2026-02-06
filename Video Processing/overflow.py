import csv

with open("rps_data.csv", newline="") as infile, \
     open("rps_data_updated.csv", "w", newline="") as outfile:

    reader = csv.DictReader(infile)
    writer = csv.DictWriter(outfile, fieldnames=reader.fieldnames)

    writer.writeheader()
    for row in reader:
        rps = float(row["rps"])
        if rps < -30:
            rps += 240
        row["rps"] = rps
        writer.writerow(row)

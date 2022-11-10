
import csv

header = ['stamp', 'v_act', 'a_act', 'a_des']
data = [0, 1, 2]

def lowpassFilter(current_value, prev_value, cutoff, dt):
  tau = 1 / (2 * math.pi * cutoff)
  a = tau / (dt + tau)
  return prev_value * a + (1 - a) * current_value


from pathlib import Path
def print_lines():
    print(Path('test.csv').read_text())

with open('test.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerow(data)
    writer.writerow(data)
    writer.writerow(data)

print_lines()

repr(Path('test.tsv').read_text())


print_lines()

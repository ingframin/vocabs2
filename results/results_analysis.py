from matplotlib import pyplot as plt

        
def read(filename):
    f = open(filename)
    raw = f.readlines()
    f.close()
    errors = set()
    err = 0
    k = ''

    data = {'No loss':[],'ADS-B':[],'Wi-Fi':[]}

    rates = set()

    for line in raw:
        ls = line.split()
        if "Error" in ls[0]:
            err = float(ls[1])
            if err < 0:
                err = 0
            continue

        if 'No' in ls[0] or 'ADS-B' in ls[0] or 'Wi-Fi' in ls[0]:
            k = ' '.join(ls)
            continue
        rates.add(float(ls[0]))
        data[k].append(float(ls[1]))
    r = list(rates)
    r.sort()
    return data, r

d,r = read('results_2d.txt')
nlp = d['No loss'][0:len(r)]
adp = d['ADS-B'][0:len(r)]
wip = d['Wi-Fi'][0:len(r)]
plt.plot(r,nlp, label='No loss')
plt.plot(r,adp, label='ADS-B')
plt.plot(r,wip, label='Wi-Fi')
plt.ax
plt.show()

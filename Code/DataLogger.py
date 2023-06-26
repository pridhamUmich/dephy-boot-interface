import os, sys
from datetime import datetime 

class DataLogger:
    def __init__(self, data_to_log, path = '', base_name = '', max_file_size = 500 * 10**6):
        
        
        self.file_base = ''
        self.file_extension = ".csv"
        
        self.max_file_size = -1
        
        self.data_filename = ''
        self.data_file = None
        self.new(data_to_log, path = path, base_name = base_name, max_file_size = max_file_size)
        
        labels_csv = DataLogger.get_labels(data_to_log)
        self.data_file.write(labels_csv)
        self.data_file.write("\n ")
        # File stays open till we want to close.
    
    def __del__(self):
        self.close()
    
    def get_free_filename(base, extension):
        data_filename = base + extension
        i = 0
        while os.path.exists(data_filename):
            i +=1
            data_filename =  base + "_"+ str(i) + extension
        return data_filename
        
    def get_labels(data_to_log):
        labels = ''
        for dict_of_data in data_to_log:
            for key in dict_of_data:
                labels +=  f"{key}," 
        return labels
    
    def get_values(data_to_log):
        values = ''
        for dict_of_data in data_to_log:
            for key in dict_of_data:
                values +=  f"{dict_of_data[key]},"    
        return values
        
    def log (self, data_to_log):
        # check file size
        if self.data_file.tell() > self.max_file_size : # tell used instead of os.path.getsize since the file is open.
            self.data_file.close()
            self.data_filename = DataLogger.get_free_filename(self.file_base, self.file_extension)
            self.data_file = open(self.data_filename, 'a')
            
            labels_csv = DataLogger.get_labels(data_to_log)
            self.data_file.write(labels_csv)
            self.data_file.write("\n ")
        
        values = DataLogger.get_values(data_to_log)
        self.data_file.write(values)
        self.data_file.write("\n ")
        
    def close(self):
        self.data_file.close()
    
    def new(self, data_to_log, path = '', base_name = '', max_file_size = 500 * 10**6):
        if self.data_file != None:
            if not self.data_file.closed:
                self.close()
            
        start_time = datetime.now()
        time_str = start_time.strftime("%Y_%m_%d_%Hh_%Mm_%Ss")
        if base_name != '':
            time_str += '_'
        
        # you need to initilize these before calling these
        if path != '' and not os.path.exists(path) : 
            os.makedirs(path)
        self.file_base = os.path.join(path, time_str + base_name)
        self.file_extension = ".csv"
        
        self.max_file_size = max_file_size
        
        self.data_filename = DataLogger.get_free_filename(self.file_base, self.file_extension)
        self.data_file = open(self.data_filename, 'a')
    
if __name__ == '__main__':
    data_1 = {
        'a' : 1,
        'b' : 2,
        'c' : False,
    }

    data_2 = {
        'd' : 3,
        'e' : 8,
    }

    data_3 = {
        'f' : False,
        'g' : 9.81,
    }

    data_full = [data_1, data_2, data_3]

    logger = DataLogger(data_full, path = 'test_folder', base_name = 'test', max_file_size = (2 * 10**3)) # max filesize = 2 kb to force second file

    while data_1['a'] <= 100:
        logger.log(data_full)
        data_1['a'] += 1
        data_full = [data_1, data_2, data_3]
    
    logger.new(data_full, path = 'test_folder', base_name = 'new', max_file_size = (2 * 10**3))
    while data_1['a'] <= 150:
        logger.log(data_full)
        data_1['a'] += 1
        data_full = [data_1, data_2, data_3]
    
        
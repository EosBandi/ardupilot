local DATA32_1 = {}
DATA32_1.id = 170
DATA32_1.fields = {
             { "type", "<B" },
             { "len", "<B" },
             { "vin", "<c10"},
             { "serial","<c10"},
             { "sysid","<B"},
             { "timers","<B"},
             { "comission_time","<H"},
             { "log_entries","<B"}
             }
return DATA32_1

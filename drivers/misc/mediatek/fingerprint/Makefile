## ranyanhao@wind-mobi.com 20160216 begin
ifeq ($(CONFIG_MTK_GF318_SUPPORT),y)
obj-y += gf318/
endif
## ranyanhao@wind-mobi.com 20160216 end

##zhenglihong@wind-mobi.com 20160430 begin
ifeq ($(CONFIG_MTK_AFS121_SUPPORT),y)
# zhaozhensen@wind-mobi.com 20160725 begin
ifeq ($(CONFIG_TRUSTKERNEL_TEE_SUPPORT),y)
obj-y += afs121/
else
# zhaozhensen@wind-mobi.com 20160725 end
obj-y += afs121tee/
endif
# zhaozhensen@wind-mobi.com 20160725 begin
endif
# zhaozhensen@wind-mobi.com 20160725 end
##zhenglihong@wind-mobi.com 20160430 end

## liukangping@wind-mobi.com 20160421 begin
ifeq ($(CONFIG_MTK_SILEAD6163),y) 
obj-y += silead6163/
endif
## liukangping@wind-mobi.com 20160421 end

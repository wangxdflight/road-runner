start /wait gradlew shadowJar

dir core\build\*.jar 
copy core\build\*.jar  ..\libs\


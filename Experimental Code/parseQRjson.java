public class HelloWorld{
    
    public static void main(String []args){
        double pattern = 0, final_x = 0, final_y = 0, final_z = 0;
        String contents = "{\"p\":1,\"x\":10.23,\"y\":-8.12,\"z\":5.45}";
        String[] multi_contents = contents.split(",");
        for (int i = 0; i < multi_contents.length; i++){
            System.out.print(multi_contents[i]);
            System.out.print(",, ");
        }
        System.out.println("");
        pattern = Double.parseDouble(multi_contents[0].split(":")[1]);
        final_x = Double.parseDouble(multi_contents[1].split(":")[1]);
        final_y = Double.parseDouble(multi_contents[2].split(":")[1]);
        final_z = Double.parseDouble(multi_contents[3].split(":")[1].split("}")[0]);
        System.out.println(pattern);
        System.out.println(final_x);
        System.out.println(final_y);
        System.out.println(final_z);
    }
}
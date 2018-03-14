package AntSimulator_EH;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;
import java.io.File;
import java.io.FileOutputStream;
import java.util.LinkedList;
import java.util.Queue;
import java.io.IOException;
/*����ǰ�᣺
 * 1:һ���ڵ�ʱʱ�̿�֪����Χ�ڵ������������ȣ������ʣ�Ҫ�ǵ��յ�����Ҳ֪���ˣ��Ͳ��У�
     2:����Ĵ���û���ŶӺʹ����ӳ٣�ֻ�д����ӳ�1��timeslot��Ҳ����˵һ��timeslotһ�����ݰ���Ȼǰ��һ�������ڵ㻺�����޴�
     3:����ͼ
     4��������Ϊ0������ɹ��ʰٷ�֮��
     
 ���ò�����1.��ʼʱ��0�Žڵ㽫�����������*40�����ϵ���������Ϊ12�� 
        2.֮��0�Žڵ㽫ÿ��10��ʱ϶����һֻ���ϣ�������
     
 ���ʣ�1.�ڵ�n��ʱ϶ʱ���ݶ�Ϊ����1��Ӧʹ�õڣ�n-1����ʱ϶��ͼ������ ���� ��2������ ��  Ӧ��ѡ�����֣�
     ��Ŀǰ�ݶ�Ϊ��һ�֣�
     2.���ĺ����Ĳ���Ȩ�أ�(���Ҫͨ��ʵ���ҳ�)
     3.�ɲ����������������ƣ���������ڵ���һ���ڵ�ʱ�����Լ����յ�Զ�˻�û���߽����ͣ�
     	��1��ֹͣ����(��Ҫ��)
     	��2���ر���һ��������һ��ʱ϶����������Ӧ��·һ�����Ϣ��(��ʱ�������ַ���)���������������ʱ��������4��֮һ
     
��֤������
       1.�����������޷�������Ϣʱ������������
	   2.����������������������ڵ���ʣ�����Ӻͺ�ƽ��ֵ��
	   3.������������ƽ������
	   4.��1000��ʵ�飬����Щ����ƽ��ֵ
	   5.�����ߣ��������ٶ�
	   
	   ע�⣺���Ʊ����������ģ�̶�
��Ҫ���ģ�
	����ڵ�ĸ�����ƣ���2000�ڵ����ߣ�3000�ڵ㸴�
   	   ����Э�飺
   	 ����������У��ʿ���ġ�˶ʿ���ģ���������
   	 û���ϸ�������֤
    
2018.2.26:
	����Э������ͬ�ڵ�֮����Դ�������
	��EH���ԵĴ������У��ص�����������ʣ�Ҳ���ǽ��ʹ������������ΪEH����ʹ������ڵ��������������ˣ�����ҲҪ���������ռ�Ԥ����ƣ�ʹ�ڵ��ռ�������������������
	
	ֵ��˼�������⣺���ڱ���Ҫʹ�����������ŵ�վ��ʵ��ĳ��Ŀ�꣬����ĳ��ָ���ϣ�������Ӧ��ĳ�������ϡ�
2018.3.7:
	���������ı�׼������10��ǰ�����Ϸ���ʧ��
	д�ļ��Ĳ�����δ���
2018.3.8:
	�ڵ�����Լ������Ļ��ƣ���Χ����ھ���С����+1����������12������12�����������仯����㲥֪ͨ��Χ�ھӡ�
	����������ʱ����1.�ڵ��յ��������ϣ�(�Ƿ���Ҫ���Ȳ�Ҫд)2.�ڵ��յ�ĳ����Χ�ڵ�����/�����֪ͨ��3.�ڵ��յ�ĳ���ھӽڵ㷢����·����Ϣ�����
	������ɢ���ٶ�Ҳ��ÿ��timeslotǰ��һ��
	֪ͨ�������ٶȺ����ݰ�һ�������ǲ��ķ�����
2018.3.9:
	��������һ��ÿ��ʱ��ƽ����·������
	       ���߶���ÿ��ʱ�̵�ǰ��·������
	����֮����1.��һ��baselineЭ��
		   2.ѧϰ�ÿ������˲���Ԥ��EH����
		3.��An Effective Energy-Harvesting-Aware Routing
		  Algorithm for WSN-based IoT Applications
		  ������������ƪ���£�
	
	
*/

class ForwardAnt{//ǰ������
	
	private static final int livetime = 12;
	private int seqno;//���к�
	private int dest;//�յ�
	private int ttl;//��������
	private Vector<Integer> visited;//���ɱ�
	private int startposition;
	private int position;//λ��
	private int lasthop;//��¼�վ�������һ�������յ�ľ���
	private boolean dead;
	
	public ForwardAnt(int seqno,int dest,int startposition){//����ǰ������
		this.seqno = seqno;
		this.dest = dest;
		this.ttl = livetime;
		this.visited = new Vector<Integer>();
		this.position = startposition;
		this.setStartposition(startposition);
		this.setDead(false);
		this.lasthop = -1;
	}
	
	public void ttlminus(){
		this.ttl--;
	}
	
	public void addvisited(){
		this.visited.add(this.position);
	}
	
	public void movetonext(int nextposition){
		this.position = nextposition;
	}
	
	public int getSeqno() {
		return seqno;
	}
	
	public void setSeqno(int seqno) {
		this.seqno = seqno;
	}
	
	public int getTtl() {
		return ttl;
	}
	
	public void setTtl(int ttl) {
		this.ttl = ttl;
	}
	
	public int getDest() {
		return dest;
	}
	
	public void setDest(int dest) {
		this.dest = dest;
	}
	
	public Vector<Integer> getVisited() {
		return visited;
	}
	
	public void setVisited(Vector<Integer> visited) {
		this.visited = visited;
	}
	
	public int getPosition() {
		return position;
	}
	
	public void setPosition(int position) {
		this.position = position;
	}

	public int getStartposition() {
		return startposition;
	}

	public void setStartposition(int startposition) {
		this.startposition = startposition;
	}
	
	public boolean isDead() {
		return dead;
	}

	public void setDead(boolean dead) {
		this.dead = dead;
	}

	public int getLasthop() {
		return lasthop;
	}

	public void setLasthop(int lasthop) {
		this.lasthop = lasthop;
	}
}

class BackwardAnt{//��������
	private int hop;//����
	private Vector<Integer> visited;
	private int roadlength;
	private int position;//λ��
	private boolean hasdead;
	private int from;
	public BackwardAnt(Vector<Integer> visit,int startposition)
	{
		this.visited = new Vector<Integer>();
		for(int i=visit.size()-1;i>=0;i--)
		{
			this.visited.add(visit.get(i));
		}
		this.hop = 0;
		this.roadlength = visit.size();//����
		this.position = startposition;
		this.setHasdead(false);
		this.from = -1;
	}
	
	public int getHop() {
		return hop;
	}
	public void setHop(int hop) {
		this.hop = hop;
	}
	public Vector<Integer> getVisited() {
		return visited;
	}
	public void setVisited(Vector<Integer> visited) {
		this.visited = visited;
	}
	public int getRoadlength() {
		return roadlength;
	}
	public void setRoadlength(int roadlength) {
		this.roadlength = roadlength;
	}
	public int getPosition() {
		return position;
	}
	public void setPosition(int position) {
		this.position = position;
	}

	public boolean isHasdead() {
		return hasdead;
	}

	public void setHasdead(boolean hasdead) {
		this.hasdead = hasdead;
	}

	public int getFrom() {
		return from;
	}

	public void setFrom(int from) {
		this.from = from;
	}
}




class Graph{//ͼ,����������ͼ
	
	private int scale;//����������Ĺ�ģ
	private List<Integer> energy;//ÿ���ڵ������
	private List<Boolean> node_enabled;//�ڵ��Ƿ�����
	private List<List<Float>> phero;//��·��Ϣ��
	private List<List<Boolean>> enabled;//��·�Ƿ�ͨ
	private List<Integer> hop;//��ǰ�ڵ㵽�յ�ľ��루��ʱ�ò��ϣ�
	private Queue<Integer> recalculate_node;

	public void showgragh()
	{
		for(int i=this.scale-1;i>=0;i--)//12\8\4\0
		{
			int k = i*this.scale-1;
			for(int j=1;j<=this.scale;j++)
			{
				k++;
				System.out.print(this.energy.get(k));
				System.out.print(" ");
			}
			System.out.print("\n");
		}
		for(int i=this.scale-1;i>=0;i--)//12\8\4\0
		{
			int k = i*this.scale-1;
			for(int j=1;j<=this.scale;j++)
			{
				k++;
				System.out.print(this.hop.get(k));
				System.out.print(" ");
			}
			System.out.print("\n");
		}
	}
	
	public void consume_enengy(int node)//����һ�����ݰ�������-5
	{
		int c = this.energy.get(node);
		this.energy.set(node, c-5);
	}
	
	public List<Integer> getEnergy() {
		return energy;
	}

	public void setEnergy(List<Integer> energy) {
		this.energy = energy;
	}

	public List<List<Float>> getPhero() {
		return phero;
	}

	public void setPhero(List<List<Float>> phero) {
		this.phero = phero;
	}

	public List<List<Boolean>> getEnabled() {
		return enabled;
	}

	public void setEnabled(List<List<Boolean>> enabled) {
		this.enabled = enabled;
	}

	public List<Integer> getHop() {
		return hop;
	}

	public void setHop(List<Integer> hop) {
		this.hop = hop;
	}

	public List<Boolean> getNode_enabled() {
		return node_enabled;
	}

	public void setNode_enabled(List<Boolean> node_enabled) {
		this.node_enabled = node_enabled;
	}
	
	public Queue<Integer> getRecalculate_node() {
		return recalculate_node;
	}

	public void setRecalculate_node(Queue<Integer> recalculate_node) {
		this.recalculate_node = recalculate_node;
	}
	
	public Graph(int scale)
	{
		this.scale = scale;
		energy = new ArrayList<Integer>();
		hop = new ArrayList<Integer>();
		phero = new ArrayList<List<Float>>();
		enabled = new ArrayList<List<Boolean>>();
		node_enabled = new ArrayList<Boolean>();
		recalculate_node = new LinkedList<Integer>();
	}
	
	public int getScale(){
		return scale;
	}
	
	public void init()//��ʼ��ͼ
	{
		for(int i=0;i<this.scale*this.scale;i++)//����ߣ��þ����ʾ�����к�ָ���к�
		{
			if(i==0 || i== this.scale*this.scale-1)
			{
				
				energy.add(999999);
			}
			else
			{
				energy.add(9999);
			}
			node_enabled.add(true);
			if(i==this.scale*this.scale-1)
			{
				hop.add(0);
			}
			else
			{
				hop.add(12);//Ĭ��Ϊʮ����
			}
			List<Float> phero_line = new ArrayList<Float>();
			List<Boolean> enabled_line = new ArrayList<Boolean>();
			phero.add(phero_line);
			enabled.add(enabled_line);
			
			for(int j=0;j<this.scale*this.scale;j++)
			{
				phero_line.add((float) 10000);
				enabled_line.add(true);
			}
		}
	}
	
	
	public Graph clonegraph()//������ܳ����⣬���ƵĶ������ڴ����Ƿ���ڣ������Ƿ���ȷ���ƣ�
	{
		Graph a = new Graph(this.scale);
		for(int i=0;i<this.scale*this.scale;i++)//����ͼ�����
		{
			a.getEnergy().add(this.energy.get(i));
			a.getHop().add(this.hop.get(i));
			a.getNode_enabled().add(this.node_enabled.get(i));
			List<Float> phero_line = new ArrayList<Float>();
			List<Boolean> enabled_line = new ArrayList<Boolean>();
			for(int j=0;j<this.scale*this.scale;j++)
			{
				phero_line.add(this.phero.get(i).get(j));
				enabled_line.add(this.enabled.get(i).get(j));
			}
			a.getPhero().add(phero_line);
			a.getEnabled().add(enabled_line);
			
		}
		return a;
	}
	
	public Vector<Integer> find_neighbor(int node)
	{
		Vector<Integer> neighbor = new Vector<Integer>();
		if(node==0)//�����0�Žڵ�
		{
			neighbor.add(1);
			neighbor.add(this.scale);
			neighbor.add(this.scale+1);
		}
		else if(node>0 && node<this.scale-1)
		{
			neighbor.add(node-1);
			neighbor.add(node+1);
			neighbor.add(node-1+this.scale);
			neighbor.add(node+1+this.scale);
			neighbor.add(node+this.scale);
		}
		else if(node == this.scale-1)
		{
			neighbor.add(node-1);
			neighbor.add(node+this.scale);
			neighbor.add(node+this.scale-1);
		}
		else if((node%this.scale==0 && node != 0)&&(node != this.scale*(this.scale-1)))
		{
			neighbor.add(node - this.scale);
			neighbor.add(node +this.scale);
			neighbor.add(node +1);
			neighbor.add(node +1+this.scale);
			neighbor.add(node +1-this.scale);
		}
		else if(node == this.scale*(this.scale-1))
		{
			neighbor.add(node-this.scale);
			neighbor.add(node +1);
			neighbor.add(node +1-this.scale);
		}
		else if(node >this.scale*(this.scale-1) && node <this.scale*this.scale-1)
		{
			neighbor.add(node+1);
			neighbor.add(node+1-this.scale);
			neighbor.add(node-1);
			neighbor.add(node-1-this.scale);
			neighbor.add(node-this.scale);
		}
		else if(node == this.scale*this.scale-1)
		{
			neighbor.add(node-1);
			neighbor.add(node-this.scale);
			neighbor.add(node-1-this.scale);
		}
		else if((node % this.scale == this.scale-1 &&node !=this.scale*this.scale-1)&&(node !=this.scale-1))
		{
			neighbor.add(node-1);
			neighbor.add(node-1+this.scale);
			neighbor.add(node+this.scale);
			neighbor.add(node-this.scale);
			neighbor.add(node-this.scale-1);
		}
		else
		{
			neighbor.add(node-1);
			neighbor.add(node+1);
			neighbor.add(node+this.scale);
			neighbor.add(node-this.scale);
			neighbor.add(node-1+this.scale);
			neighbor.add(node-1-this.scale);
			neighbor.add(node+1+this.scale);
			neighbor.add(node+1-this.scale);
		}
		return neighbor;
	}
	
	public int selectnexthop(int node,Vector<Integer> visited,int dest,int hop){//���ĺ�������ǰ������ѡ����һ���ڵ㣬���ڵĺܼ�
		
		//���ҳ������ھӽڵ㣬����֤�ھӽڵ��ǲ���enabled,visited,�ٱȽ�
		int nexthop = 9999;//9999Ĭ��Ϊ��Ч�ڵ�
		
		Vector<Integer> neighbor = find_neighbor(node);
		
		if(neighbor.contains(dest)==true)
		{
			if(enabled.get(node).get(dest)==true)
			{
				nexthop = dest;
				return nexthop;
			}
		}
		int high_neighbor = -1;
		Vector<Integer> high_num = new Vector<Integer>();
		high_num.addElement(9999);
		for(int i=0;i<neighbor.size();i++)
		{
			if(visited.contains(neighbor.get(i))==false)
			{
				if(enabled.get(node).get(neighbor.get(i))==true)
				{
					int u = energy.get(neighbor.get(i))/1000;
					//int u = energy.get(neighbor.get(i));
					if ((int)(phero.get(node).get(neighbor.get(i)) * u) > high_neighbor)
					{
						high_num.removeAllElements();
						high_num.add(neighbor.get(i));
						
						high_neighbor = (int) (phero.get(node).get(neighbor.get(i)) * u);
					}
					else if((int)(phero.get(node).get(neighbor.get(i))*u)==high_neighbor)
					{
						high_num.add(neighbor.get(i));
					}
				}
			}
			
		}
		
		int x = (int)(Math.random()*high_num.size());
		nexthop = (int)high_num.get(x);
		return nexthop; 
	}
	
	public void battery_change()//���ߺͻ��ѽڵ�
	{
		/*���ߣ�������ýڵ㷢�͵���·ȫ��ʧЧ�������ɸýڵ㷢�͵���·Ҳȫ��ʧЧ,��Ӧ����Ϣ��Ũ�Ƚ�Ϊ0*/
		for(int i=0;i<this.scale*this.scale;i++)
		{
			if(energy.get(i)<2000 && node_enabled.get(i)==true)//С��2000ʱ������
			{
				node_enabled.set(i, false);
				Vector<Integer> neighbor = this.find_neighbor(i);
				for(int j=0;j<neighbor.size();j++)
				{
					enabled.get(i).set(neighbor.get(j), false);
					phero.get(i).set(neighbor.get(j), (float)0);
					enabled.get(neighbor.get(j)).set(i, false);
					phero.get(neighbor.get(j)).set(i, (float)0);
					if(node_enabled.get(neighbor.get(j))==true)
					{
						recalculate_node.offer(neighbor.get(j));
					}
				}
			}
			/*���ѻ��ƻ�û�����룬����д*/
			
			
			
		}
	}
	
	public void recalculate_hop(int node)
	{
		Vector<Integer> neighbor = find_neighbor(node);
		int smallest = hop.get(node)-1;
		for(int i=0;i<neighbor.size();i++)
		{	
			if(energy.get(neighbor.get(i))>2000)//Ĭ����·����Ч
			{
				if(smallest > hop.get(neighbor.get(i)))
				{
					smallest = hop.get(neighbor.get(i));
				}
			}
		}
		hop.set(node, smallest+1);
	}
	
}
public class AntSimulator_EH {
	
	public static final int simulate_time = 60000;//ģ��ʱ϶����
	public static final int netscale = 7;//�����ģ
	public static final int destnode = 48;//�յ�
	public static final int startnum = 45;//��ʼʱ��������������
	public static final int interval = 3;//�������ϵ�ʱ϶�������
	public static final int livetime = 12;//���ݰ�������ʱ��
	public static int seqno;
	public static int timeslot;
	public static Graph research_graph;
 	public static List<ForwardAnt> f_ant;
	public static List<BackwardAnt> b_ant;
	public static int receive;
	public static int hopnumber;
	public static List<Integer> dead_forward_ant;
	public static List<Integer> node_update;//ʵ���ϴ���һ�������ڵ�֪ͨ
	
	public static void generate_forward_ant(int seqno,int dest,int startposition)
	{//����ǰ�����ϲ��������뵽��Ӫ������
		ForwardAnt a = new ForwardAnt(seqno,dest,startposition);
		f_ant.add(a);
	}
	
	public static void move_ant(int time,Graph g,FileOutputStream file)//g�����и���,����������һ��
	{	
		//�ȱ����Ϊ�ڵ���������������仯,ֻ��ĳ�ڵ��hop�ڸ��ֱ仯�������ٴι㲥��ɢ
		Map<Integer,Integer> hop1 = new HashMap<Integer,Integer>();
		for(Integer y:research_graph.getRecalculate_node())
		{
			if(hop1.containsKey(y)==false)
			{
				hop1.put(y,research_graph.getHop().get(y));
			}
		}
		int n = research_graph.getRecalculate_node().size();
		for(int i=0;i<n;i++)//�������
		{
			research_graph.recalculate_hop(research_graph.getRecalculate_node().element());
			research_graph.getRecalculate_node().poll();
		}
		for(Integer j:hop1.keySet())
		{
			if(hop1.get(j)!=research_graph.getHop().get(j))
			{
				Vector<Integer> c = research_graph.find_neighbor(j);
				for(int b = 0;b<c.size();b++)
				{
					if(research_graph.getNode_enabled().get(c.get(b))==true)
					{
						research_graph.getRecalculate_node().offer(c.get(b));
					}
				}
			}
		}		
		
		for(int i = 0;i<f_ant.size();i++)//ǰ�����ϵ��ƶ�
		{
			if(f_ant.get(i).getPosition()==f_ant.get(i).getStartposition())//ʼ��
			{
				f_ant.get(i).ttlminus();//ttl��һ
				f_ant.get(i).addvisited();//�����ڵ������ɱ�
				research_graph.consume_enengy(f_ant.get(i).getPosition());//�ڵ��������٣�����ע���������
				int nexthop = g.selectnexthop(f_ant.get(i).getPosition(),f_ant.get(i).getVisited(),f_ant.get(i).getDest(),g.getHop().get(f_ant.get(i).getPosition()));		//
				f_ant.get(i).setLasthop(g.getHop().get(f_ant.get(i).getPosition()));
				if(nexthop == 9999)//�����һ��û��
				{
					System.out.println("�Ҳ�����һ����");
					//��¼����������
					dead_forward_ant.add(f_ant.get(i).getSeqno());
					//removeforwardant.add(i);
					f_ant.get(i).setDead(true);
				}
				else//�ҵ�����һ��
				{
					f_ant.get(i).movetonext(nexthop);
				}
				
			}
			else if(f_ant.get(i).getPosition()!=f_ant.get(i).getStartposition() && f_ant.get(i).getPosition()!=f_ant.get(i).getDest())
			{//��ת
				if(f_ant.get(i).getTtl()==0)//ttl = 0;
				{
					System.out.println("ǰ������������");
					//��¼����������
					dead_forward_ant.add(f_ant.get(i).getSeqno());
					f_ant.get(i).setDead(true);
				}
				else//ttl!=0
				{
					f_ant.get(i).ttlminus();//ttl��һ
					
					research_graph.consume_enengy(f_ant.get(i).getPosition());
					if(g.getHop().get(f_ant.get(i).getPosition())>f_ant.get(i).getLasthop())//ǰ�����Ϸ���Խ��ԽԶ��
					{
						System.out.println("Զ�ˣ�");
						int k = f_ant.get(i).getVisited().size();
						float x = research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).get(f_ant.get(i).getPosition());
						research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).set(f_ant.get(i).getPosition(),(float) ((float)x*0.25));
						research_graph.consume_enengy(f_ant.get(i).getPosition());
					}
					
					else if(g.getHop().get(f_ant.get(i).getPosition())==f_ant.get(i).getLasthop())//ǰ�����Ϸ�����һ������һ��һ��
					{
						System.out.println("��ȣ�");
						int k = f_ant.get(i).getVisited().size();
						float x = research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).get(f_ant.get(i).getPosition());
						research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).set(f_ant.get(i).getPosition(),(float) ((float)x*0.5));
						research_graph.consume_enengy(f_ant.get(i).getPosition());
					}
					
					f_ant.get(i).setLasthop(g.getHop().get(f_ant.get(i).getPosition()));
					f_ant.get(i).addvisited();//�����ڵ������ɱ�
					int nexthop = g.selectnexthop(f_ant.get(i).getPosition(),f_ant.get(i).getVisited(),f_ant.get(i).getDest(),g.getHop().get(f_ant.get(i).getPosition()));
					if(nexthop == 9999)//�����һ��û�У��ͰѶ�Ӧ��·����Ϣ����Ϊ��0��
					{
						System.out.println("�Ҳ�����һ����");
						//��¼����������
						dead_forward_ant.add(f_ant.get(i).getSeqno());
						int k = f_ant.get(i).getVisited().size();
						research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).set(f_ant.get(i).getPosition(),(float) 0);
						research_graph.consume_enengy(f_ant.get(i).getPosition());
						f_ant.get(i).setDead(true);
					}
					else//�ҵ�����һ��
					{
						f_ant.get(i).movetonext(nexthop);
					}
				}
			}
			else if(f_ant.get(i).getPosition()==f_ant.get(i).getDest())//�����յ�
			{
				if(f_ant.get(i).getTtl()==0)//ttl = 0;
				{
					System.out.println("ǰ������������");
					//��¼����������
					dead_forward_ant.add(f_ant.get(i).getSeqno());
					f_ant.get(i).setDead(true);
				}
				else//ttl!=0
				{
					System.out.println(f_ant.get(i).getSeqno()+"�ɹ������յ㣡·����");
					for(int j=0;j<f_ant.get(i).getVisited().size();j++)
					{
						System.out.print(f_ant.get(i).getVisited().get(j));
						System.out.print(" ");
					}
					System.out.print("\n");
					/*����Ҫ������������*/
					BackwardAnt new_backant = new BackwardAnt(f_ant.get(i).getVisited(),f_ant.get(i).getDest());
					b_ant.add(new_backant);
					
					f_ant.get(i).setDead(true);
				}
			}
		}
		
		boolean hasdead = false;
		//����Ҫ�Ƴ�������ǰ������
		do{
			hasdead = false;
			for(int h =0;h<f_ant.size();h++)
			{
				if(f_ant.get(h).isDead())
				{
					f_ant.remove(h);
					hasdead = true;
					break;
				}
			}
		}while(hasdead == true);
		
		for(int j = 0;j<b_ant.size();j++)//�ƶ���������,��������Ҳ�������������⣬�����ǵ����յ�ʱ����������
		{
			if(b_ant.get(j).getHop()==0)//�����ʼ��
			{
				int nexthop = b_ant.get(j).getVisited().get(0);
				int c = b_ant.get(j).getHop()+1;
				b_ant.get(j).setHop(c);
				b_ant.get(j).setFrom(b_ant.get(j).getPosition());
				research_graph.consume_enengy(b_ant.get(j).getPosition());
				b_ant.get(j).getVisited().remove(0);
				b_ant.get(j).setPosition(nexthop);
			}
			else if(b_ant.get(j).getHop()>0 && b_ant.get(j).getHop()<b_ant.get(j).getRoadlength())
			{//��ת
				int nexthop = b_ant.get(j).getVisited().get(0);
				int hop = b_ant.get(j).getHop();
				int c = b_ant.get(j).getHop()+1;
				b_ant.get(j).setHop(c);
				int from = b_ant.get(j).getFrom();
				research_graph.consume_enengy(b_ant.get(j).getPosition());
				//���������Ϣ��
				Vector<Integer> neighbor = g.find_neighbor(b_ant.get(j).getPosition());
				for(int k=0;k<neighbor.size();k++)
				{//ֻ�е���·״̬Ϊtrue��ʱ��Ÿ�����Ϣ��,ע��������research_graph
					if(research_graph.getEnabled().get(b_ant.get(j).getPosition()).get(neighbor.get(k))==true)
					{
						float b = research_graph.getPhero().get(b_ant.get(j).getPosition()).get(neighbor.get(k));
						research_graph.getPhero().get(b_ant.get(j).getPosition()).set((int)neighbor.get(k),(float) ((float)b*0.2));
					}
				}
				if(hop<research_graph.getHop().get(b_ant.get(j).getPosition()))
				{//��������
					research_graph.getHop().set(b_ant.get(j).getPosition(), hop);
				}
				float v = research_graph.getPhero().get(b_ant.get(j).getPosition()).get(from);
				//v = v + 1000000/(research_graph.getHop().get(b_ant.get(j).getPosition()));
				v = v+1000/hop;
				research_graph.getPhero().get(b_ant.get(j).getPosition()).set(from, v);
				b_ant.get(j).setFrom(b_ant.get(j).getPosition());
				b_ant.get(j).getVisited().remove(0);
				b_ant.get(j).setPosition(nexthop);
			}
			else if(b_ant.get(j).getHop()==b_ant.get(j).getRoadlength())
			{
				System.out.println("�������ϵ���ڵ㣡");
				receive++;
				hopnumber = hopnumber + b_ant.get(j).getRoadlength();
				//����д�ļ�
				try {
					file.write((String.valueOf(time)+" ").getBytes());
					file.write((String.valueOf(b_ant.get(j).getRoadlength())+"\n").getBytes());
				} catch (IOException e1) {
					e1.printStackTrace();
				}
				
				
				//seqno++;
				//generate_forward_ant(seqno,destnode,0);
				int hop = b_ant.get(j).getHop();
				int from = b_ant.get(j).getFrom();
				try {
					file.write((String.valueOf(timeslot)+" "+String.valueOf(hop)+"\n").getBytes());
				} catch (IOException e) {
					e.printStackTrace();
				}
				//���������Ϣ��
				Vector<Integer> neighbor = g.find_neighbor(b_ant.get(j).getPosition());
				for(int k=0;k<neighbor.size();k++)
				{//ֻ�е���·״̬Ϊtrue��ʱ��Ÿ�����Ϣ��,ע��������research_graph
					if(research_graph.getEnabled().get(b_ant.get(j).getPosition()).get(neighbor.get(k))==true)
					{
						float b = research_graph.getPhero().get(b_ant.get(j).getPosition()).get(neighbor.get(k));
						research_graph.getPhero().get(b_ant.get(j).getPosition()).set((int)neighbor.get(k),(float) ((float)b*0.2));
					}
				}
				if(hop<research_graph.getHop().get(b_ant.get(j).getPosition()))
				{//��������
					research_graph.getHop().set(b_ant.get(j).getPosition(), hop);
				}
				float v = research_graph.getPhero().get(b_ant.get(j).getPosition()).get(from);
				//v = v + 1000000/(research_graph.getHop().get(b_ant.get(j).getPosition()));
				v = v+1000/hop;
				research_graph.getPhero().get(b_ant.get(j).getPosition()).set(from, v);
				b_ant.get(j).setHasdead(true);
			}
		}
		boolean hasdead2 = false;
		//����Ҫ�Ƴ������ĺ�������
		do{
			hasdead2 = false;
			for(int h =0;h<b_ant.size();h++)
			{
				if(b_ant.get(h).isHasdead())
				{
					b_ant.remove(h);
					hasdead2 = true;
					break;
				}
			}
		}while(hasdead2 == true);
		
		
		research_graph.battery_change();
		
		
		
	}
	
	public static boolean checknetdead(int scale)//����Ƿ�������ǰ����������10��������
	{
		/*�ȶ�dead_forward_ant�е�����������*/
		
		Collections.sort(dead_forward_ant);
		
		boolean isdead = false;
		int i;
		for(i=0;i<dead_forward_ant.size();i++)
		{
			int start = dead_forward_ant.get(i);
			if(i+50<dead_forward_ant.size())
			{
				if(dead_forward_ant.get(i+50)==start+50)
				{
					isdead = true;
					break;
				}
			}
			else
			{
				break;
			}
		}
		return isdead;
	}
	
	public static void main(String[] args)
    {
		
		FileOutputStream out = null;
		try{
			out = new FileOutputStream(new File("D:\\data1.txt"));
		}
		catch(Exception e){
			
		}
		hopnumber = 0;
		seqno = -1;
		receive = -1;
        research_graph = new Graph(netscale);//��ʼ�����缰�����ģ��4*4����
        research_graph.init();
        f_ant=new ArrayList<ForwardAnt>();
        b_ant=new ArrayList<BackwardAnt>();
        dead_forward_ant = new ArrayList<Integer>();
        node_update = new ArrayList<Integer>();
    	//���濪ʼģ�⣬��ÿһ��ʱ϶��ʼʱ���Ȳ������ϣ�ǰ���򣩣�ÿֻ����������Ӧ����
    	
    	for(timeslot = 0;timeslot<simulate_time;timeslot++)
    	{
    		if(timeslot == 0)//����ǵ�һ��ʱ϶������40ֻ,����ǰ�ƶ�
    		{
    			for(int i=1;i<=startnum;i++)
    			{
    				seqno++;
    				generate_forward_ant(seqno,destnode,0);
    			}
    			Graph g = research_graph.clonegraph();//��¡���и���
    			
    			move_ant(timeslot,g,out);//�����ƶ������������и���������������ԭ��
    		}
    		else if(timeslot>20 && timeslot %10 == 0)
    		{
    			seqno++;
    			generate_forward_ant(seqno,destnode,0);
    			Graph g = research_graph.clonegraph();
    			move_ant(timeslot,g,out);
    		}
    		else if(timeslot>20 && timeslot % 10!=0) 
    		{
    			Graph g = research_graph.clonegraph();
    			move_ant(timeslot,g,out);
    		}
    		if(checknetdead(netscale)==true)
    		{
    			break;
    		}
    	}
        research_graph.showgragh();
        System.out.print("������ʱ�䣺");
        System.out.println(timeslot);
        System.out.println("�յ��ĺ���������Ŀ��"+String.valueOf(receive));
        System.out.print("���հٷֱȣ�");
        System.out.print((float)receive/seqno*100);
        System.out.println("%");
        System.out.print("����ƽ��������");
        System.out.println((float)hopnumber/receive);
        System.out.println("�������϶�����ֵ��");
        for(int i=0;i<dead_forward_ant.size();i++)
        {
        	System.out.print(dead_forward_ant.get(i));
        	System.out.print(" ");
        }
    }	
}
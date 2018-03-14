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
/*假设前提：
 * 1:一个节点时时刻刻知道周围节点的情况（能量等）（疑问：要是到终点跳数也知道了，就不行）
     2:分组的传输没有排队和处理延迟（只有传播延迟1个timeslot，也就是说一个timeslot一个数据包必然前进一跳），节点缓存无限大
     3:有向图
     4：丢包率为0，传输成功率百分之百
     
 设置参数：1.开始时刻0号节点将随机发送蚂蚁*40，蚂蚁的生存周期为12跳 
        2.之后0号节点将每隔10个时隙产生一只蚂蚁，并发送
     
 疑问：1.在第n个时隙时，暂定为：（1）应使用第（n-1）个时隙的图像数据 或者 （2）不管 ，  应当选择哪种？
     （目前暂定为第一种）
     2.核心函数的参数权重？(这个要通过实验找出)
     3.可不可以引入这样机制：如果蚂蚁在到达一个节点时候发现自己离终点远了或没有走近，就：
     	（1）停止传输(需要吗？)
     	（2）回报上一跳，在下一个时隙其蒸发掉对应链路一半的信息素(暂时采用这种方案)，当发现跳数相等时，蒸发掉4分之一
     
验证参数：
       1.网络寿命（无法发出信息时，网络寿命）
	   2.网络能量消耗情况（各个节点所剩能量加和和平均值）
	   3.传输数据所需平均跳数
	   4.做1000次实验，求这些数字平均值
	   5.画曲线，看收敛速度
	   
	   注意：控制变量，网络规模固定
需要做的：
	引入节点的复活机制；（2000节点休眠，3000节点复活）
   	   书上协议：
   	 几个顶级高校博士论文、硕士论文（找综述）
   	 没有严格理论推证
    
2018.2.26:
	能量协作，不同节点之间可以传递能量
	在EH特性的传感网中，重点是提高吞吐率，也就是降低传输的跳数，因为EH特性使得网络节点的能量大大增加了，但是也要引入能量收集预测机制，使节点收集能量多于消耗能量。
	
	值得思考的问题：现在必须要使考虑问题的落脚点站在实现某个目标，改善某个指标上，而并非应用某个方法上。
2018.3.7:
	网络死亡的标准是连续10个前进蚂蚁发送失败
	写文件的操作还未添加
2018.3.8:
	节点更新自己跳数的机制：周围存活邻居最小跳数+1，假若超过12，则还是12，假如跳数变化，则广播通知周围邻居。
	更新跳数的时机：1.节点收到后向蚂蚁；(是否需要？先不要写)2.节点收到某个周围节点死亡/复活的通知；3.节点收到某个邻居节点发来的路由信息变更包
	跳数扩散的速度也是每个timeslot前进一格
	通知包传播速度和数据包一样，但是不耗费能量
2018.3.9:
	画：曲线一：每个时刻平均的路径长度
	       曲线二：每个时刻当前的路径长度
	当务之急：1.找一个baseline协议
		   2.学习用卡尔曼滤波器预测EH速率
		3.读An Effective Energy-Harvesting-Aware Routing
		  Algorithm for WSN-based IoT Applications
		  （电脑中有这篇文章）
	
	
*/

class ForwardAnt{//前行蚂蚁
	
	private static final int livetime = 12;
	private int seqno;//序列号
	private int dest;//终点
	private int ttl;//生存周期
	private Vector<Integer> visited;//禁忌表
	private int startposition;
	private int position;//位置
	private int lasthop;//记录刚经过的上一跳距离终点的距离
	private boolean dead;
	
	public ForwardAnt(int seqno,int dest,int startposition){//生成前进蚂蚁
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

class BackwardAnt{//后行蚂蚁
	private int hop;//跳数
	private Vector<Integer> visited;
	private int roadlength;
	private int position;//位置
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
		this.roadlength = visit.size();//跳数
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




class Graph{//图,这里是网格图
	
	private int scale;//正方形网格的规模
	private List<Integer> energy;//每个节点的能量
	private List<Boolean> node_enabled;//节点是否休眠
	private List<List<Float>> phero;//链路信息素
	private List<List<Boolean>> enabled;//链路是否导通
	private List<Integer> hop;//当前节点到终点的距离（暂时用不上）
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
	
	public void consume_enengy(int node)//发送一个数据包，能量-5
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
	
	public void init()//初始化图
	{
		for(int i=0;i<this.scale*this.scale;i++)//有向边，用矩阵表示，由行号指向列号
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
				hop.add(12);//默认为十二跳
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
	
	
	public Graph clonegraph()//这里可能出问题，复制的对象在内存中是否存在？存在是否正确复制？
	{
		Graph a = new Graph(this.scale);
		for(int i=0;i<this.scale*this.scale;i++)//复制图的情况
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
		if(node==0)//如果是0号节点
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
	
	public int selectnexthop(int node,Vector<Integer> visited,int dest,int hop){//核心函数，给前行蚂蚁选择下一跳节点，现在的很简单
		
		//先找出所有邻居节点，再验证邻居节点是不是enabled,visited,再比较
		int nexthop = 9999;//9999默认为无效节点
		
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
	
	public void battery_change()//休眠和唤醒节点
	{
		/*休眠：所有向该节点发送的链路全部失效，所有由该节点发送的链路也全部失效,对应的信息素浓度降为0*/
		for(int i=0;i<this.scale*this.scale;i++)
		{
			if(energy.get(i)<2000 && node_enabled.get(i)==true)//小于2000时候休眠
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
			/*唤醒机制还没有引入，下面写*/
			
			
			
		}
	}
	
	public void recalculate_hop(int node)
	{
		Vector<Integer> neighbor = find_neighbor(node);
		int smallest = hop.get(node)-1;
		for(int i=0;i<neighbor.size();i++)
		{	
			if(energy.get(neighbor.get(i))>2000)//默认链路还有效
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
	
	public static final int simulate_time = 60000;//模拟时隙数量
	public static final int netscale = 7;//网络规模
	public static final int destnode = 48;//终点
	public static final int startnum = 45;//开始时产生的蚂蚁数量
	public static final int interval = 3;//产生蚂蚁的时隙间隔数量
	public static final int livetime = 12;//数据包的生存时间
	public static int seqno;
	public static int timeslot;
	public static Graph research_graph;
 	public static List<ForwardAnt> f_ant;
	public static List<BackwardAnt> b_ant;
	public static int receive;
	public static int hopnumber;
	public static List<Integer> dead_forward_ant;
	public static List<Integer> node_update;//实际上代表一次死亡节点通知
	
	public static void generate_forward_ant(int seqno,int dest,int startposition)
	{//产生前行蚂蚁并把它加入到运营队列中
		ForwardAnt a = new ForwardAnt(seqno,dest,startposition);
		f_ant.add(a);
	}
	
	public static void move_ant(int time,Graph g,FileOutputStream file)//g是现有副本,用来决定下一跳
	{	
		//先变更因为节点死亡引起的跳数变化,只有某节点的hop在该轮变化，才能再次广播扩散
		Map<Integer,Integer> hop1 = new HashMap<Integer,Integer>();
		for(Integer y:research_graph.getRecalculate_node())
		{
			if(hop1.containsKey(y)==false)
			{
				hop1.put(y,research_graph.getHop().get(y));
			}
		}
		int n = research_graph.getRecalculate_node().size();
		for(int i=0;i<n;i++)//变更跳数
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
		
		for(int i = 0;i<f_ant.size();i++)//前行蚂蚁的移动
		{
			if(f_ant.get(i).getPosition()==f_ant.get(i).getStartposition())//始发
			{
				f_ant.get(i).ttlminus();//ttl减一
				f_ant.get(i).addvisited();//将本节点加入禁忌表
				research_graph.consume_enengy(f_ant.get(i).getPosition());//节点能量减少，这里注意操作对象
				int nexthop = g.selectnexthop(f_ant.get(i).getPosition(),f_ant.get(i).getVisited(),f_ant.get(i).getDest(),g.getHop().get(f_ant.get(i).getPosition()));		//
				f_ant.get(i).setLasthop(g.getHop().get(f_ant.get(i).getPosition()));
				if(nexthop == 9999)//如果下一跳没有
				{
					System.out.println("找不到下一跳！");
					//记录死亡的蚂蚁
					dead_forward_ant.add(f_ant.get(i).getSeqno());
					//removeforwardant.add(i);
					f_ant.get(i).setDead(true);
				}
				else//找到了下一跳
				{
					f_ant.get(i).movetonext(nexthop);
				}
				
			}
			else if(f_ant.get(i).getPosition()!=f_ant.get(i).getStartposition() && f_ant.get(i).getPosition()!=f_ant.get(i).getDest())
			{//中转
				if(f_ant.get(i).getTtl()==0)//ttl = 0;
				{
					System.out.println("前行蚂蚁死亡！");
					//记录死亡的蚂蚁
					dead_forward_ant.add(f_ant.get(i).getSeqno());
					f_ant.get(i).setDead(true);
				}
				else//ttl!=0
				{
					f_ant.get(i).ttlminus();//ttl减一
					
					research_graph.consume_enengy(f_ant.get(i).getPosition());
					if(g.getHop().get(f_ant.get(i).getPosition())>f_ant.get(i).getLasthop())//前行蚂蚁发现越走越远了
					{
						System.out.println("远了！");
						int k = f_ant.get(i).getVisited().size();
						float x = research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).get(f_ant.get(i).getPosition());
						research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).set(f_ant.get(i).getPosition(),(float) ((float)x*0.25));
						research_graph.consume_enengy(f_ant.get(i).getPosition());
					}
					
					else if(g.getHop().get(f_ant.get(i).getPosition())==f_ant.get(i).getLasthop())//前行蚂蚁发现这一跳跟上一跳一样
					{
						System.out.println("相等！");
						int k = f_ant.get(i).getVisited().size();
						float x = research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).get(f_ant.get(i).getPosition());
						research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).set(f_ant.get(i).getPosition(),(float) ((float)x*0.5));
						research_graph.consume_enengy(f_ant.get(i).getPosition());
					}
					
					f_ant.get(i).setLasthop(g.getHop().get(f_ant.get(i).getPosition()));
					f_ant.get(i).addvisited();//将本节点加入禁忌表
					int nexthop = g.selectnexthop(f_ant.get(i).getPosition(),f_ant.get(i).getVisited(),f_ant.get(i).getDest(),g.getHop().get(f_ant.get(i).getPosition()));
					if(nexthop == 9999)//如果下一跳没有，就把对应链路的信息素设为“0”
					{
						System.out.println("找不到下一跳！");
						//记录死亡的蚂蚁
						dead_forward_ant.add(f_ant.get(i).getSeqno());
						int k = f_ant.get(i).getVisited().size();
						research_graph.getPhero().get(f_ant.get(i).getVisited().get(k-1)).set(f_ant.get(i).getPosition(),(float) 0);
						research_graph.consume_enengy(f_ant.get(i).getPosition());
						f_ant.get(i).setDead(true);
					}
					else//找到了下一跳
					{
						f_ant.get(i).movetonext(nexthop);
					}
				}
			}
			else if(f_ant.get(i).getPosition()==f_ant.get(i).getDest())//到达终点
			{
				if(f_ant.get(i).getTtl()==0)//ttl = 0;
				{
					System.out.println("前行蚂蚁死亡！");
					//记录死亡的蚂蚁
					dead_forward_ant.add(f_ant.get(i).getSeqno());
					f_ant.get(i).setDead(true);
				}
				else//ttl!=0
				{
					System.out.println(f_ant.get(i).getSeqno()+"成功到达终点！路径：");
					for(int j=0;j<f_ant.get(i).getVisited().size();j++)
					{
						System.out.print(f_ant.get(i).getVisited().get(j));
						System.out.print(" ");
					}
					System.out.print("\n");
					/*下面要构建后向蚂蚁*/
					BackwardAnt new_backant = new BackwardAnt(f_ant.get(i).getVisited(),f_ant.get(i).getDest());
					b_ant.add(new_backant);
					
					f_ant.get(i).setDead(true);
				}
			}
		}
		
		boolean hasdead = false;
		//下面要移除死亡的前进蚂蚁
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
		
		for(int j = 0;j<b_ant.size();j++)//移动后向蚂蚁,后向蚂蚁也存在死亡的问题，当它们到达终点时，就死亡了
		{
			if(b_ant.get(j).getHop()==0)//如果是始发
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
			{//中转
				int nexthop = b_ant.get(j).getVisited().get(0);
				int hop = b_ant.get(j).getHop();
				int c = b_ant.get(j).getHop()+1;
				b_ant.get(j).setHop(c);
				int from = b_ant.get(j).getFrom();
				research_graph.consume_enengy(b_ant.get(j).getPosition());
				//下面更新信息素
				Vector<Integer> neighbor = g.find_neighbor(b_ant.get(j).getPosition());
				for(int k=0;k<neighbor.size();k++)
				{//只有当链路状态为true的时候才更新信息素,注意这里用research_graph
					if(research_graph.getEnabled().get(b_ant.get(j).getPosition()).get(neighbor.get(k))==true)
					{
						float b = research_graph.getPhero().get(b_ant.get(j).getPosition()).get(neighbor.get(k));
						research_graph.getPhero().get(b_ant.get(j).getPosition()).set((int)neighbor.get(k),(float) ((float)b*0.2));
					}
				}
				if(hop<research_graph.getHop().get(b_ant.get(j).getPosition()))
				{//更新跳数
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
				System.out.println("后向蚂蚁到达节点！");
				receive++;
				hopnumber = hopnumber + b_ant.get(j).getRoadlength();
				//下面写文件
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
				//下面更新信息素
				Vector<Integer> neighbor = g.find_neighbor(b_ant.get(j).getPosition());
				for(int k=0;k<neighbor.size();k++)
				{//只有当链路状态为true的时候才更新信息素,注意这里用research_graph
					if(research_graph.getEnabled().get(b_ant.get(j).getPosition()).get(neighbor.get(k))==true)
					{
						float b = research_graph.getPhero().get(b_ant.get(j).getPosition()).get(neighbor.get(k));
						research_graph.getPhero().get(b_ant.get(j).getPosition()).set((int)neighbor.get(k),(float) ((float)b*0.2));
					}
				}
				if(hop<research_graph.getHop().get(b_ant.get(j).getPosition()))
				{//更新跳数
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
		//下面要移除死亡的后退蚂蚁
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
	
	public static boolean checknetdead(int scale)//检测是否死亡的前进蚂蚁中有10个连续的
	{
		/*先对dead_forward_ant中的数字做排序*/
		
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
        research_graph = new Graph(netscale);//初始化网络及网络规模，4*4网格
        research_graph.init();
        f_ant=new ArrayList<ForwardAnt>();
        b_ant=new ArrayList<BackwardAnt>();
        dead_forward_ant = new ArrayList<Integer>();
        node_update = new ArrayList<Integer>();
    	//下面开始模拟，在每一个时隙开始时，先产生蚂蚁（前后向），每只蚂蚁再做相应动作
    	
    	for(timeslot = 0;timeslot<simulate_time;timeslot++)
    	{
    		if(timeslot == 0)//如果是第一个时隙，产生40只,并向前移动
    		{
    			for(int i=1;i<=startnum;i++)
    			{
    				seqno++;
    				generate_forward_ant(seqno,destnode,0);
    			}
    			Graph g = research_graph.clonegraph();//克隆现有副本
    			
    			move_ant(timeslot,g,out);//蚂蚁移动的依据是现有副本，操作对象是原本
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
        System.out.print("网络存活时间：");
        System.out.println(timeslot);
        System.out.println("收到的后向蚂蚁数目："+String.valueOf(receive));
        System.out.print("接收百分比：");
        System.out.print((float)receive/seqno*100);
        System.out.println("%");
        System.out.print("蚂蚁平均跳数：");
        System.out.println((float)hopnumber/receive);
        System.out.println("死亡蚂蚁队列中值：");
        for(int i=0;i<dead_forward_ant.size();i++)
        {
        	System.out.print(dead_forward_ant.get(i));
        	System.out.print(" ");
        }
    }	
}